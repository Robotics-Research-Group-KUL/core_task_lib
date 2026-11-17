--  Copyright (c) 2025 KU Leuven, Belgium
--
--  Author: Santiago Iregui
--  email: <santiago.iregui@kuleuven.be>
--
--  GNU Lesser General Public License Usage
--  Alternatively, this file may be used under the terms of the GNU Lesser
--  General Public License version 3 as published by the Free Software
--  Foundation and appearing in the file LICENSE.LGPLv3 included in the
--  packaging of this file. Please review the following information to
--  ensure the GNU Lesser General Public License version 3 requirements
--  will be met: https://www.gnu.org/licenses/lgpl.html.
-- 
--  This program is distributed in the hope that it will be useful,
--  but WITHOUT ANY WARRANTY; without even the implied warranty of
--  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
--  GNU Lesser General Public License for more details.

require("context")
require("geometric")
-- worldmodel=require("worldmodel")
require("math")

reqs = require("task_requirements")
 
task_description = [[
Keeps the pose of a task frame (a robot frame) relative to a moving base frame equal to the relative pose they had at the start of the task, so that the task frame follows the motion of the base frame (e.g. a tracked object).
Position and orientation are handled separately (activate_linear / activate_angular): when activated, the relative position resp. orientation w.r.t. base_frame is kept at its initial value; when deactivated, the position resp. orientation of the task frame is kept fixed at its initial value w.r.t. the robot base frame (root frame of the robot model), ignoring the motion of base_frame.
Input channel: a Frame named as the parameter base_frame (default "base_frame"), giving the base frame pose w.r.t. the robot base frame, typically provided by a PoseInputHandler or TFInputHandler.
The task runs until stopped externally, unless execution_time > 0, in which case it exits after execution_time seconds. Outputs: time and the position (x_tf, y_tf, z_tf) [m] of the task frame w.r.t. the robot base frame.]]

-- ========================================= PARAMETERS ===================================
 
param = reqs.parameters(task_description,{
    reqs.params.string({name="task_frame", description="Name of the robot frame (as defined in the robot model, e.g. URDF link or tcp_frame) whose pose is kept constant relative to base_frame", default = "tcp_frame", required=false}),
    reqs.params.string({name="base_frame", description="Name of the input channel (Frame) with the pose of the moving base frame, expressed w.r.t. the robot base frame. The task frame keeps its initial pose relative to this frame. It must be provided by an input handler, e.g. a PoseInputHandler or a TFInputHandler", default = "base_frame", required=false}),
    reqs.params.scalar({name="execution_time", description="Time [s] after which the task stops automatically (exit monitor). If execution_time <= 0 (default 0), the task does not stop by itself and runs until stopped externally", default = 0, required=false, minimum = -1}),
    reqs.params.bool({name="activate_linear", description="If true, the position of the task frame relative to base_frame (expressed in base_frame) is kept at its initial value, i.e. the task frame follows the motion of base_frame. If false, the position of the task frame is kept fixed at its initial value w.r.t. the robot base frame", default = true, required=true}),
    reqs.params.bool({name="activate_angular", description="If true, the orientation of the task frame relative to base_frame is kept at its initial value, i.e. the task frame rotates along with base_frame. If false, the orientation of the task frame is kept fixed w.r.t. the robot base frame", default = false, required=true}),
    reqs.params.scalar({name="K_relative_position", description="Feedback gain [1/s] of the constraint that keeps the relative position w.r.t. base_frame (used when activate_linear is true). Larger values correct deviations faster (more aggressive tracking); too large values can cause oscillations or saturate the joint velocities", default = 2, required=false, minimum = 0}),
    reqs.params.scalar({name="K_relative_rotation", description="Feedback gain [1/s] of the constraint that keeps the relative orientation w.r.t. base_frame (used when activate_angular is true). Larger values correct deviations faster; too large values can cause oscillations or saturate the joint velocities", default = 1, required=false, minimum = 0}),
    reqs.params.scalar({name="K_fixed_position", description="Feedback gain [1/s] of the constraint that keeps the position of the task frame fixed w.r.t. the robot base frame (used when activate_linear is false). Larger values correct deviations faster", default = 4, required=false, minimum = 0}),
    reqs.params.scalar({name="K_fixed_rotation", description="Feedback gain [1/s] of the constraint that keeps the orientation of the task frame fixed w.r.t. the robot base frame (used when activate_angular is false). Larger values correct deviations faster", default = 4, required=false, minimum = 0}),
    reqs.params.scalar({name="weight_position", description="Weight (dimensionless) of the position constraint (relative or fixed, depending on activate_linear) w.r.t. the other constraints of the same priority. Only relative weights matter: a higher weight makes this constraint dominate when the constraints conflict", default = 1, required=false, minimum = 0}),
    reqs.params.scalar({name="weight_rotation", description="Weight (dimensionless) of the orientation constraint (relative or fixed, depending on activate_angular) w.r.t. the other constraints of the same priority. Only relative weights matter: a higher weight makes this constraint dominate when the constraints conflict", default = 1, required=false, minimum = 0}),

})

-- TODO: Change order of quaterinions in the skill.
-- TODO: Check what happens when quaterion is not valid.
-- TODO: Change tcp_2_tf.

-- ======================================== Robot model requirements ========================================
robot = reqs.robot_model({--This function loads the robot model and checks that all required frames are available
    param.get("task_frame"), --The frame is selected as a parameter, to make the skill even more reusable
})
robot_joints = robot.robot_joints
task_frame = robot.getFrame(param.get("task_frame"))

base_frame   = ctx:createInputChannelFrame(param.get("base_frame"))

T_tcp_bf = inv(base_frame)*task_frame --Transformation from TCP to base_frame 

T_tcp_bf_target = initial_value(time, T_tcp_bf) --Initial transformation from TCP to base_frame 


-- ========================== CONSTRAINT SPECIFICATION =================================

if param.get("activate_linear") then

    Constraint{
        context = ctx,
        name    = "maintain_relative_posisition",
        expr    = origin(T_tcp_bf) - origin(T_tcp_bf_target),
        K       = param.get("K_relative_position"),
        weight  = param.get("weight_position"),
        priority= 2
    }
else
    Constraint{
        context = ctx,
        name    = "keep_translation_constant",
        expr    = origin(task_frame),
        target  = initial_value(time, origin(task_frame)),
        K       = param.get("K_fixed_position"),
        weight  = param.get("weight_position"),
        priority= 2
    };
end



if param.get("activate_angular") then
    
    Constraint{
        context = ctx,
        name    = "maintain_relative_rotation",
        expr    = inv(rotation(T_tcp_bf_target))*rotation(T_tcp_bf),
        K       = param.get("K_relative_rotation"),
        weight  = param.get("weight_rotation"),
        priority= 2
    }
else
    Constraint{
        context = ctx,
        name    = "keep_rotation_constant",
        expr    = rotation(task_frame)*initial_value(time, rotation(task_frame)),
        K       = param.get("K_fixed_rotation"),
        weight  = param.get("weight_rotation"),
        priority= 2
    };

end


-- =========================== MONITOR ============================================
-- Monitor{
--         context=ctx,
--         name='finish_after_motion',
--         upper=0.0,
--         actionname='exit',
--         expr=time-get_duration(mp) - constant(1)
-- }

-- Monitor{
--     context=ctx,
--     name='finish_and_trigger_console',
--     upper=0.0,
--     actionname='debug',
--     expr=time- constant(1)
-- }


-- Monitor{
--     context=ctx,
--     name='portevent_test',
--     upper=0.0,
--     actionname='portevent',
--     argument = "test_event",
--     expr=time-0.5
-- }

-- Monitor{
--     context=ctx,
--     name='event_test',
--     upper=0.0,
--     actionname='event',
--     argument = "test_event",
--     expr=time-0.5
-- }

execution_time = param.get("execution_time")

if(execution_time>0) then 
    Monitor{
        context=ctx,
        name='finish_after_motion_ended',
        upper=0.0,
        actionname='exit',
        expr=time-constant(execution_time)
    }
end



ctx:setOutputExpression("time",time)
ctx:setOutputExpression("x_tf",coord_x(origin(task_frame)))
ctx:setOutputExpression("y_tf",coord_y(origin(task_frame)))
ctx:setOutputExpression("z_tf",coord_z(origin(task_frame)))



-- ============================== OUTPUT THROUGH PORTS===================================
-- ctx:setOutputExpression("x_tf",coord_x(origin(task_frame)))
-- ctx:setOutputExpression("y_tf",coord_y(origin(task_frame)))
-- ctx:setOutputExpression("z_tf",coord_z(origin(task_frame)))
--
-- roll_tf,pitch_tf,yaw_tf = getRPY(rotation(task_frame))
-- ctx:setOutputExpression("roll_tf",roll_tf)
-- ctx:setOutputExpression("pitch_tf",pitch_tf)
-- ctx:setOutputExpression("yaw_tf",yaw_tf)