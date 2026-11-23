--  Copyright (c) 2025 KU Leuven, Belgium
--
--  Author: Erwin Aertbeliën, Santiago Iregui
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


-- ========================================= PARAMETERS ===================================
task_description = "Teleoperation with a 6D joystick (a.k.a. spacemouse) along a virtual fixture shaped as a horn: the origin of the task frame is kept on a quadratic B-spline that starts at the base of the fixture (fixture_base) and ends at a point of its top disk (radius, at height); the end point (feature variables p_x, p_y) and the progress along the spline (feature variable s) are free, so the joystick velocities move the tool along and around the horn. Optionally, the orientation of the task frame is kept tangent to the spline (its z-axis along the spline)."

param = reqs.parameters(task_description,{
    reqs.params.scalar({name="linear_scale", description="Scales the magnitude of the linear velocity coming from the joystick", default = 0.3, required=false}),
    reqs.params.scalar({name="angular_scale", description="Scales the magnitude of the angular velocity coming from the joystick", default = 0.3, required=false}),
    reqs.params.string({name="task_frame", description="Name of frame used to control the robot in cartesian space", default = "tcp_frame", required=false}),
    reqs.params.bool({name="activate_linear", description="If true, the linear velocities of the joystick will control the linear velocities of the defined frame", default = true, required=true}),
    reqs.params.bool({name="activate_angular", description="If true, the angular velocities of the joystick will control the angular velocities of the defined frame", default = false, required=true}),
    reqs.params.scalar({name="execution_time", description="Time (seconds) that the task should run before stopping. If zero, the execution will not stop automatically (i.e. no monitor will be triggered)", default = 0, required=false, minimum = 0}),
    reqs.params.bool({name="activate_angular_fixture", description="If true, the z-axis of the task frame is kept tangent to the spline of the fixture", default = true, required=true}),
    reqs.params.scalar({name="radius", description="Radius [m] of the top disk of the fixture: the end point of the spline stays inside it", default = 0.2, required=true}),
    reqs.params.scalar({name="height", description="Height [m] of the fixture, along the z-axis of fixture_base", default = 0.5, required=true}),
    reqs.params.string({name="joystick_input_channel", description="Name of the input channel (type twist) from which the joystick velocities are read: the varname of the input handler that provides it (e.g. a TwistInputHandler subscribed to /spacenav/twist)", default = "joystick_input", required=false}),
    reqs.params.array({name="fixture_base", type=reqs.array_types.number, default={0.0, -0.680, 0.180, 0.0, 0.0, 0.0, 1.0}, description="Pose of the base frame of the fixture w.r.t. the robot base frame: [x,y,z,qx,qy,qz,qw] (position [m] and quaternion). The fixture starts at its origin and goes along its z-axis", required=true, minItems = 7, maxItems = 7})
})
linear_scale    = constant(param.get("linear_scale"))
angular_scale    = constant(param.get("angular_scale"))
execution_time = param.get("execution_time")   

fixture_base = param.get("fixture_base")
x_coordinate   = constant(fixture_base[1])
y_coordinate   = constant(fixture_base[2])
z_coordinate   = constant(fixture_base[3])
q_i            = constant(fixture_base[4])
q_j            = constant(fixture_base[5])
q_k            = constant(fixture_base[6])
q_real         = constant(fixture_base[7])
quat = quaternion(q_real,vector(q_i,q_j,q_k))
fixture_R      = toRot(quat)
fixture_P      = vector(x_coordinate,y_coordinate,z_coordinate)
fixture_pose   = frame(fixture_R, fixture_P)


-- ======================================== Robot model requirements ========================================
robot = reqs.robot_model({ --This function loads the robot model and checks that all required frames are available
    param.get("task_frame"), --The frame is selected as a parameter, to make the skill even more reusable
    --Add all frames that are required by the task specification
}) 
robot_joints = robot.robot_joints
task_frame = robot.getFrame(param.get("task_frame"))

-- ========================================= Variables coming from topic input handlers ===================================
joystick_input   = ctx:createInputChannelTwist(param.get("joystick_input_channel"))


-- ================================ Fixture =================================
bs     = require("bspline")
degree = 2          -- number of control points = number of knots - degree + 1 = 3
radius = param.get("radius")
L      = param.get("height")

knots  = {0, 0,  1, 1}

p_x = Variable{context=ctx, name="p_x", vartype="feature"}
p_y = Variable{context=ctx, name="p_y", vartype="feature"}
s = Variable{context=ctx, name="s", vartype="feature", initial = 0.5}

cp_x = {constant(0), constant(0), p_x}
cp_y = {constant(0), constant(0), p_y}
cp_z = {constant(0), constant(L/2), constant(L) }

spline_x = bs.generate_spline(s, knots, cp_x, degree)
spline_y = bs.generate_spline(s, knots, cp_y, degree)
spline_z = bs.generate_spline(s, knots, cp_z, degree)

fixture =  { fixture_pose*vector(cp_x[1],cp_y[1],cp_z[1]),
             fixture_pose*vector(cp_x[2],cp_y[2],cp_z[2]),
             fixture_pose*vector(cp_x[3],cp_y[3],cp_z[3]) }
-- 3rd control point within a disk of given radius
Constraint{
    context      = ctx, 
    name         = "cubic_horn", 
    expr         = p_x * p_x + p_y * p_y, 
    target_upper = radius*radius, 
    K            = 4, 
    priority     = 1
}
-- do not go outside the top and bottom of the horn
Constraint{
    context       = ctx, 
    name          = "progress_limits",
    expr          = s, 
    target_lower  = 0.01, 
    target_upper  = 0.99, 
    K             = 4,
    priority      = 1
}
-- the task frame should follow the spline fixture
target = fixture_pose * vector(spline_x,spline_y,spline_z)
Constraint{
    context       = ctx,
    name          = "robot_on_fixture", 
    expr          = origin(task_frame), 
    target        = target, 
    K             = 4, 
    weight        = 10, 
    priority      = 2
}

if param.get("activate_angular_fixture") then
    -- a new spline corresponding to the derivative of the orginal spline (..._d)
    spline_xd = bs.generate_spline_d(s, knots, cp_x, degree)
    spline_yd = bs.generate_spline_d(s, knots, cp_y, degree)
    spline_zd = bs.generate_spline_d(s, knots, cp_z, degree)
    -- compute normalized tangent from these derivatives
    -- there is a trick here: make_constant ensures that the orientation will not influence
    -- the control points and progress, but the control points and progress will still influence 
    -- the orientation.  make_constant(..) can also be used to "cut" influences.
    tangent = make_constant(normalized( vector(spline_xd,spline_yd,spline_zd)  ))

    -- projection of tangent on x- and y-axis of the task frame should be zero.
    Constraint{
        context       = ctx, 
        name          = "fixture_orientation",
        expr          = dot(tangent, unit_x(rotation(task_frame))),
        target        = 0,
        K             = 4, 
        weight        = 10, 
        priority      = 2    
    }
    Constraint{
        context       = ctx, 
        name          = "fixture_orientation",
        expr          = dot(tangent, unit_y(rotation(task_frame))),
        target        = 0,
        K             = 4, 
        weight        = 10, 
        priority      = 2    
    }
end


-- ============================ Joystick constraints =====================================

if param.get("activate_linear") then
    desired_vel_x = coord_x(transvel(joystick_input))*linear_scale
    desired_vel_y = coord_y(transvel(joystick_input))*linear_scale
    desired_vel_z = coord_z(transvel(joystick_input))*linear_scale
    -- Translation velocities
    Constraint{
        context = ctx,
        name    = "x_velocity",
        expr    = coord_x(origin(task_frame)),
        target  = desired_vel_x*time,
        K       = 0,
        weight  = 1,
        priority= 2
    };
    Constraint{
        context = ctx,
        name    = "y_velocity",
        expr    = coord_y(origin(task_frame)),
        target  = desired_vel_y*time,
        K       = 0,
        weight  = 1,
        priority= 2
    };
    Constraint{
        context = ctx,
        name    = "z_velocity",
        expr    = coord_z(origin(task_frame)),
        target  = desired_vel_z*time,
        K       = 0,
        weight  = 1,
        priority= 2
    };

end

if param.get("activate_angular") then

    desired_omega_x = coord_x(rotvel(joystick_input))*angular_scale
    desired_omega_y = coord_y(rotvel(joystick_input))*angular_scale
    desired_omega_z = coord_z(rotvel(joystick_input))*angular_scale

    -- Orientation velocities
    Constraint{
        context = ctx,
        name    = "x_angular",
        expr    = coord_x(getRotVec(rotation(task_frame))) - desired_omega_x*time,
        K       = 0,
        weight  = 1,
        priority= 2
    };
    Constraint{
        context = ctx,
        name    = "y_angular",
        expr    = coord_y(getRotVec(rotation(task_frame))) - desired_omega_y*time,
        K       = 0,
        weight  = 1,
        priority= 2
    };
    Constraint{
        context = ctx,
        name    = "z_angular",
        expr    = coord_z(getRotVec(rotation(task_frame))) - desired_omega_z*time,
        K       = 0,
        weight  = 1,
        priority= 2
    };
end

quat_tf = toQuat(rotation(task_frame))

ctx:setOutputExpression("time",time)
ctx:setOutputExpression("x_tf",coord_x(origin(task_frame)))
ctx:setOutputExpression("y_tf",coord_y(origin(task_frame)))
ctx:setOutputExpression("z_tf",coord_z(origin(task_frame)))
ctx:setOutputExpression("qx_tf",coord_x(vec(quat_tf)))
ctx:setOutputExpression("qy_tf",coord_y(vec(quat_tf)))
ctx:setOutputExpression("qz_tf",coord_z(vec(quat_tf)))
ctx:setOutputExpression("qw_tf",scalar(quat_tf))


if (execution_time > 0) then
    Monitor{
        context=ctx,
        name='finish_after_motion_ended',
        upper=0.0,
        actionname='exit',
        expr=time-constant(execution_time)
    }
end

-- ============================== OUTPUT THROUGH PORTS===================================
-- ctx:setOutputExpression("x_tf",coord_x(origin(task_frame)))
-- ctx:setOutputExpression("y_tf",coord_y(origin(task_frame)))
-- ctx:setOutputExpression("z_tf",coord_z(origin(task_frame)))
--
-- roll_tf,pitch_tf,yaw_tf = getRPY(rotation(task_frame))
-- ctx:setOutputExpression("roll_tf",roll_tf)
-- ctx:setOutputExpression("pitch_tf",pitch_tf)
-- ctx:setOutputExpression("yaw_tf",yaw_tf)
