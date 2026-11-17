--  Copyright (c) 2025 KU Leuven, Belgium
--
--  Author: Santiago Iregui
--  email: <santiago.iregui@kuleuven.be>
-- 
-- Code made based on Erwin Aertbeliën's code. 
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
Moves a subset of the robot joints (the joint names given in robot_joints) from their current values to the target joint values (target_joints); the other joints are not commanded by this task (they remain still unless other constraints move them).
Each selected joint follows a trapezoidal velocity profile from its initial to its target value, limited by maxvel/maxacc; the profiles are synchronized so that all selected joints start and finish at the same time. The joints track these profiles with feedback constraints.
No input channels are used. The task exits time_margin seconds (default 2 s) after the planned duration of the motion. Outputs: time, the positions of all robot joints (jpos1, jpos2, ...) and the tracking errors of the moved joints (tracking_error_<joint name>).]]

param = reqs.parameters(task_description,{
    reqs.params.scalar({name="maxvel", description="Maximum joint velocity [rad/s] (or [m/s] for prismatic joints) of the trapezoidal profile, applied to every moved joint. The profiles of all joints are synchronized, so only the joint with the longest motion reaches this limit. Always given in SI units, independently of the units parameter", default = 0.1, required=true, maximum = 0.5}),
    reqs.params.scalar({name="maxacc", description="Maximum joint acceleration [rad/s^2] (or [m/s^2] for prismatic joints) of the trapezoidal profile, applied to every moved joint. Always given in SI units, independently of the units parameter", default = 0.1, required=true, maximum = 0.5}),
    reqs.params.array({name="target_joints", type=reqs.array_types.number, default={0.0, 0.0, 0.0}, description="Target values of the joints listed in robot_joints, in the same order (same length as robot_joints). Expressed in radians or degrees according to the units parameter (prismatic joints in meters; with units=degrees every value is multiplied by pi/180, so use radians when prismatic joints are included)", required=true, minimum = -360, maximum=360, minItems = 1}),
    reqs.params.array({name="robot_joints", type=reqs.array_types.string, default={"joint_1", "joint_2", "joint_3"}, description="Names of the robot joints (as defined in the robot model) to be moved with the trapezoidal velocity profile. Must have the same length as target_joints and fewer elements than the total number of robot joints", required=true, minItems = 1}),
    reqs.params.enum({name="units", type=reqs.enum_types.string, default="radians", description="Units of the angles given in target_joints: radians or degrees (degrees are converted to radians). It does not affect maxvel/maxacc", required=false, accepted_vals = {"degrees","radians"}}),
    reqs.params.scalar({name="K_joint", description="Feedback gain [1/s] of the joint trajectory tracking constraints (same for all moved joints). Larger values reduce the tracking error w.r.t. the trapezoidal profile faster; too large values can cause oscillations or saturate the joint velocities", default = 3, required=false, minimum = 0}),
    reqs.params.scalar({name="time_margin", description="Extra time [s] that the task keeps running (holding the target) after the planned duration of the trapezoidal profile has elapsed, before the exit monitor finishes the task. Gives the joints time to settle at the target", default = 2, required=false, minimum = 0}),
})

robot = reqs.robot_model({
    -- "tcp_frame",
    -- "forearm",
    --Add all frames that are required by the task specification
})


-- ========================================= PARAMETERS ===================================
maxvel    = constant(param.get("maxvel"))
maxacc    = constant(param.get("maxacc"))

target_joints = param.get("target_joints")
robot_joints = param.get("robot_joints")


if #robot.robot_joints < #target_joints then
    error("The number of target_joints (" .. tostring(#target_joints) .. ") must be less than or equal to the number of specified robot.robot_joints (" .. tostring(#robot.robot_joints) ..  ")")
end

if #robot_joints ~= #target_joints then
    error("The number of robot_joints (" .. tostring(#robot_joints) .. ") and the specified number of target_joints (" .. tostring(#target_joints) ..  "), both defined as a parameter, must coincide")
end


units = param.get("units")
-- print("units: ",units)
target_joints = reqs.adapt_to_units(target_joints,units)

-- ========================================= VELOCITY PROFILE ===================================

mp = create_motionprofile_trapezoidal()
mp:setProgress(time)
current_jnt = {} -- current joint value


for i=1,#robot_joints do
    current_jnt[i]   = ctx:getScalarExpr(robot_joints[i])
    mp:addOutput( initial_value(time, current_jnt[i]), constant(target_joints[i]), maxvel, maxacc)
end




-- The following creates a trapezoidal velocity profile from the initial value of each angle, towards the target angle. It checks whether the joint is continuous or bounded,
-- and if it is continuous it takes the shortest path towards the angle. This makes the skill generic to any type of robot (e.g. the Kinova).
-- The old version used the above commented method, which is the one that is explained in the etasl tutorial.
-- print("kasdjaksjdhkahskdhajkshdjkahsjkhdkjahsdhka")
-- for i=1,#robot.robot_joints do
--     current_jnt[i]   = ctx:getScalarExpr(robot.robot_joints[i])
--     local theta_init = initial_value(time, current_jnt[i])
--     local theta_final_raw = target_joints[i]
--     print(theta_final_raw)
--     local difference_theta = cached(acos(cos(theta_init)*cos(theta_final_raw)+sin(theta_init)*sin(theta_final_raw))) --Shortest angle between two unit vectors (basic formula: 'cos(alpha) = dot(a,b)'. where a and b are two unit vectors)
--     local error_difference_theta = cached(acos(cos(theta_init + difference_theta)*cos(theta_final_raw)+sin(theta_init + difference_theta)*sin(theta_final_raw))) --Shortest angle computation also. If the sign is correct, it should be zero
--     local delta_theta = cached(conditional(error_difference_theta - constant(1e-5) ,constant(-1)*difference_theta,difference_theta)) --determines the proper sign to rotate the initial angle

--     local is_continuous = ctx:createInputChannelScalar("continuous_j"..i,0)--TODO: In the next release we will be able to obtain this directly from the urdf
--     local final_angle = cached(conditional(constant(-1)*abs(is_continuous),theta_final_raw,theta_init + delta_theta)) -- Only 0 is interpreted as bounded
--     mp:addOutput( theta_init, make_constant(final_angle) , maxvel, maxacc)
-- end

duration = get_duration(mp)
-- print(duration:value())

-- ========================= CONSTRAINT SPECIFICATION ========================

tgt         = {} -- target value
tracking_error = {}
for i=1,#robot_joints do
    tgt[i]        = get_output_profile(mp,i-1)
    Constraint{
        context=ctx,
        name="joint_trajectory_"..robot_joints[i],
        expr= current_jnt[i] - tgt[i] ,
        priority = 2,
        K=param.get("K_joint")
    };
    tracking_error[i] = current_jnt[i] - tgt[i]

    -- Constraint{
    --     context=ctx,
    --     name="joint_trajectory"..i,
    --     expr= current_jnt[i] - initial_value(time, current_jnt[i]),
    --     priority = 2,
    --     K=1
    -- };


    -- Constraint{
    --     context=ctx,
    --     name="joint_trajectory"..i,
    --     expr= current_jnt[i] - target_joints[i] ,
    --     priority = 2,
    --     K=1
    -- };

    -- Constraint{
    --     context=ctx,
    --     name="joint_trajectory"..i,
    --     expr= current_jnt[i]*time ,
    --     priority = 2,
    --     K=0
    -- };

    
end

--    Constraint{
--         context=ctx,
--         name="joint_trajectory6",
--         expr= current_jnt[6] - 20*3.1416/180 ,
--         priority = 2,
--         K=1
--     };

-- =================================== MONITOR TO FINISH THE MOTION ========================

Monitor{
        context=ctx,
        name='finish_after_motion_ended',
        upper=0.0,
        actionname='exit',
        expr=time-duration -constant(param.get("time_margin"))
}

-- Monitor {
--     context = ctx,
--     name    = "time_elapsed",
--     expr    = time,
--     upper   = 1.0,
--     actionname = "print",
--     argument = "addtional argument"
-- }




ctx:setOutputExpression("time",time)


for i=1,#robot.robot_joints do
    ctx:setOutputExpression("jpos"..i,ctx:getScalarExpr(robot.robot_joints[i]))
end


for i=1,#robot_joints do
    ctx:setOutputExpression("tracking_error_"..robot_joints[i],tracking_error[i])
end