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
Moves a task frame of the robot to a desired absolute pose (desired_pose) expressed w.r.t. the robot base frame (root frame of the robot model).
The position follows a straight line from the initial to the desired position, and the orientation rotates about a single fixed axis from the initial to the desired orientation. Both are timed by one synchronized trapezoidal velocity profile limited by maxvel/maxacc (the rotation is converted to a distance with the equivalent radius eq_r), so they start and finish together. The task frame tracks this moving target with feedback constraints.
No input channels are used. The task exits when the position error w.r.t. the desired pose is below error_pos_th and the orientation error is below error_rot_th. Outputs: time and the position (x_tf, y_tf, z_tf) [m] of the task frame w.r.t. the robot base frame.]]

-- ========================================= PARAMETERS ===================================
 
param = reqs.parameters(task_description,{
    reqs.params.scalar({name="maxvel", description="Maximum velocity [m/s] of the trapezoidal motion profile. It limits the translational speed of the task frame along the path, and also the rotational motion, expressed as angle*eq_r (i.e. the maximum angular velocity is maxvel/eq_r [rad/s])", default = 0.1, required=true, maximum = 0.5}),
    reqs.params.scalar({name="maxacc", description="Maximum acceleration [m/s^2] of the trapezoidal motion profile, for the translation and for the rotation expressed as angle*eq_r (i.e. the maximum angular acceleration is maxacc/eq_r [rad/s^2])", default = 0.1, required=true, maximum = 0.5}),
    reqs.params.scalar({name="eq_r", description="Equivalent radius [m] used to convert the rotation angle [rad] into an equivalent distance (angle*eq_r) so that rotation and translation share the same maxvel/maxacc limits. A larger eq_r makes the rotation slower (max angular velocity maxvel/eq_r)", default = 0.08, required=false}),
    reqs.params.scalar({name="error_pos_th", description="Position error threshold [m] of the exit monitor: the task finishes when the distance between the task frame and the final target position is below this value (and the orientation error is below error_rot_th)", default = 0.0005, required=false}),
    reqs.params.scalar({name="error_rot_th", description="Orientation error threshold [rad] of the exit monitor: the task finishes when the rotation angle between the task frame and the final target orientation is below this value (and the position error is below error_pos_th)", default = 0.01, required=false}),
    reqs.params.string({name="task_frame", description="Name of the robot frame (as defined in the robot model, e.g. URDF link or tcp_frame) that is moved in Cartesian space", default = "tcp_frame", required=false}),
    reqs.params.array({name="desired_pose", type=reqs.array_types.number, default={0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0}, description="Desired (absolute) pose of the task frame w.r.t. the robot base frame (root frame of the robot model), as [x,y,z,qx,qy,qz,qw]: position x,y,z [m] followed by the orientation as a unit quaternion with the scalar part last (qw)", required=true, minItems = 7, maxItems = 7}),
    reqs.params.scalar({name="K_position", description="Feedback gain [1/s] of the constraint that makes the task frame position follow the generated straight-line path. Larger values reduce the tracking error faster; too large values can cause oscillations or saturate the joint velocities", default = 3, required=false, minimum = 0}),
    reqs.params.scalar({name="K_rotation", description="Feedback gain [1/s] of the constraint that makes the task frame orientation follow the generated rotation profile. Larger values reduce the tracking error faster; too large values can cause oscillations or saturate the joint velocities", default = 3, required=false, minimum = 0}),
    reqs.params.scalar({name="weight_position", description="Weight (dimensionless) of the position tracking constraint relative to the other constraints of the same priority (e.g. the orientation constraint). Only relative weights matter: a higher weight makes position tracking dominate when the constraints conflict", default = 1, required=false, minimum = 0}),
    reqs.params.scalar({name="weight_rotation", description="Weight (dimensionless) of the orientation tracking constraint relative to the other constraints of the same priority (e.g. the position constraint). Only relative weights matter: a higher weight makes orientation tracking dominate when the constraints conflict", default = 1, required=false, minimum = 0}),
})

-- TODO: Change order of quaterinions in the skill.
-- TODO: Check what happens when quaterion is not valid.
-- TODO: Change tcp_2_tf.

-- ======================================== Robot model requirements ========================================
robot = reqs.robot_model({--This function loads the robot model and checks that all required frames are available
    param.get("task_frame"), --The frame is selected as a parameter, to make the skill even more reusable
    -- "forearm"
    -- "tcp_frame"
    --Add all frames that are required by the task specification
})
robot_joints = robot.robot_joints
task_frame = robot.getFrame(param.get("task_frame"))
 
-- ========================================= PARAMETERS ===================================
maxvel    = constant(param.get("maxvel"))
maxacc    = constant(param.get("maxacc"))
eqradius  = constant(param.get("eq_r"))

error_pos_th = constant(param.get("error_pos_th"))
error_rot_th = constant(param.get("error_rot_th"))

desired_pose = param.get("desired_pose")

x_coordinate   = constant(desired_pose[1])
y_coordinate   = constant(desired_pose[2])
z_coordinate   = constant(desired_pose[3])
q_i            = constant(desired_pose[4])
q_j            = constant(desired_pose[5])
q_k            = constant(desired_pose[6])
q_real         = constant(desired_pose[7])



-- compute orientation from quaternion
quat = quaternion(q_real,vector(q_i,q_j,q_k))
target_R = toRot(quat)
target_P = vector(x_coordinate,y_coordinate,z_coordinate)
target_pose = frame(target_R, target_P)

-- =============================== INITIAL POSE ==============================
startpose = initial_value(time, task_frame)
startpos  = origin(startpose)
startrot  = rotation(startpose)

-- =========================== VELOCITY PROFILE ============================================
eps=constant(1E-14)
function close_to_zero( e, yes_expr, no_expr)
    return cached( conditional( e - eps, no_expr, conditional( -e+eps,  no_expr, yes_expr)) )
end

function normalize( v )
    n  = cached( norm(v) )
    vn = cached( close_to_zero(n, vector(constant(1),constant(0),constant(0)), v/n) )
    return vn,n
end

-- compute distances for displacements and rotations:
diff                    = cached(target_P-startpos)
diff, distance          = normalize( diff )

diff_rot                = cached(  getRotVec( inv(startrot)*target_R )) -- eq. axis of rotation for rotation from start to end:w
diff_rot, angle         = normalize( diff_rot )


-- plan trapezoidal motion profile in function of time:
mp = create_motionprofile_trapezoidal()
mp:setProgress(time)
mp:addOutput(constant(0), distance, maxvel, maxacc)
mp:addOutput(constant(0), angle*eqradius, maxvel, maxacc)
d  = get_output_profile(mp,0)            -- progression in distance
r  = get_output_profile(mp,1)/eqradius   -- progression in distance_rot (i.e. rot*eqradius)

-- =========================== TARGET POSE ============================================

targetpos = startpos + diff*d
targetrot = startrot*rotVec(diff_rot,r)

target    = frame(targetrot,targetpos)

-- ========================== CONSTRAINT SPECIFICATION =================================

Constraint{
    context = ctx,
    name    = "follow_path_rotation",
    expr    = inv(rotation(target))*rotation(task_frame),
    K       = param.get("K_rotation"),
    weight  = param.get("weight_rotation"),
    priority= 2
}

Constraint{
    context = ctx,
    name    = "follow_path_position",
    expr    = origin(target) - origin(task_frame),
    K       = param.get("K_position"),
    weight  = param.get("weight_position"),
    priority= 2
}

-- =========================== MONITOR ============================================
--Error calculation
local rot_vec = getRotVec(inv(rotation(task_frame))*target_R)
local diff_rot_error
local angle_error
diff_rot_error, angle_error = normalize(rot_vec)

local error_pos = norm(target_P-origin(task_frame))
local error_orient = angle_error
local error = conditional(error_pos-error_pos_th, constant(0), constant(1))*conditional(error_orient-error_rot_th, constant(0), constant(1)) -- Returns zero if the error is too big

Monitor{
    context=ctx,
    name='finish_after_motion',
    upper = 0.8, -- The error, that comes from a conditional, is binary (0 or 1)
    actionname = "exit",
    expr=error
}

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