--  Copyright (c) 2025 KU Leuven, Belgium
--
--  Author: Santiago Iregui
--  email: <santiago.iregui@kuleuven.be>
-- 
-- Code made based on Cristian Vergara's and Erwin Aertbeliën's code. 
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
require("libexpressiongraph_spline")
reqs = require("task_requirements")

task_description = [[
Makes a task frame of the robot follow a cubic spline path fitted through position points read from a CSV file, starting from the pose of the task frame at the start of the task.
The CSV file has 4 columns (separated by commas, spaces or tabs) and no header: the first column is the normalized path coordinate s, increasing from 0 (start) to 1 (end); the other three are the X, Y, Z position offsets [m] of the path w.r.t. the initial position of the task frame, along the axes of the robot base frame (root frame of the robot model). The first point should be (0, 0, 0, 0) so that the path starts at the current position.
The progress along the path is a path coordinate s (feature variable) that follows a trapezoidal velocity profile in s, limited by maxvel/maxacc (expressed in units of s, i.e. 1/s and 1/s^2) and bounded to [0, 1]. Meanwhile the orientation rotates progressively (proportional to s) about the initial Z axis of the task frame by rotation_z_end (default 1.5708 rad, i.e. about 90 degrees).
No input channels are used. The task exits when s reaches the end of the path (s = 1). Outputs: time, the position (x_tf, y_tf, z_tf) [m] of the task frame w.r.t. the robot base frame, and progress_variable (s).]]

-- ========================================= PARAMETERS ===================================

param = reqs.parameters(task_description,{
    reqs.params.scalar({name="maxvel", description="Maximum rate of progress of the normalized path coordinate s [1/s] (s goes from 0 to 1 over the whole path), i.e. the inverse of the minimum time to traverse the path. The resulting Cartesian speed [m/s] is approximately maxvel times the length of the path", default = 0.1, required=true, maximum = 0.5}),
    reqs.params.scalar({name="maxacc", description="Maximum acceleration of the normalized path coordinate s [1/s^2] used by the trapezoidal profile. The resulting Cartesian acceleration [m/s^2] is approximately maxacc times the length of the path", default = 0.1, required=true, maximum = 0.5}),
    reqs.params.string({name="task_frame", description="Name of the robot frame (as defined in the robot model, e.g. URDF link or tcp_frame) that follows the spline", default = "tcp_frame", required=false}),
    reqs.params.string({name="csv_file_path", description="Path of the CSV file (extension .csv) with the points that define the spline (4 columns: s in [0,1], X, Y, Z offsets [m]). Either an absolute path or a path of the form $[ros_package_name]/relative/path, which is resolved w.r.t. the share directory of that ROS 2 package", default = "$[crospi_application_template]/task_specifications/libraries/core_task_lib/task_specifications/misc/example_spline.csv", pattern=".*\\.csv$", required=true}),
    reqs.params.scalar({name="rotation_z_end", description="Total rotation angle [rad] of the task frame about its own initial Z axis at the end of the path. The rotation is applied progressively, proportionally to the path coordinate s. Use 0 to keep the initial orientation", default = 3.1416/2, required=false}),
    reqs.params.scalar({name="K_position", description="Feedback gain [1/s] of the constraint that makes the task frame position follow the spline. Larger values reduce the tracking error faster; too large values can cause oscillations or saturate the joint velocities", default = 4, required=false, minimum = 0}),
    reqs.params.scalar({name="K_rotation", description="Feedback gain [1/s] of the constraint that makes the task frame orientation follow the orientation profile. Larger values reduce the tracking error faster; too large values can cause oscillations or saturate the joint velocities", default = 4, required=false, minimum = 0}),
    reqs.params.scalar({name="weight_position", description="Weight (dimensionless) of the position tracking constraint relative to the other constraints of the same priority (orientation tracking and path progress constraints). Only relative weights matter: a higher weight makes position tracking dominate when constraints conflict", default = 10, required=false, minimum = 0}),
    reqs.params.scalar({name="weight_rotation", description="Weight (dimensionless) of the orientation tracking constraint relative to the other constraints of the same priority (position tracking and path progress constraints). Only relative weights matter: a higher weight makes orientation tracking dominate when constraints conflict", default = 10, required=false, minimum = 0}),
    reqs.params.scalar({name="weight_progress_profile", description="Weight (dimensionless) of the constraint that imposes the trapezoidal velocity profile on the path coordinate s. A higher weight enforces the profile more strictly against the tracking constraints (a lower weight lets the progress slow down when the robot cannot track the path)", default = 20, required=false, minimum = 0}),
    reqs.params.scalar({name="weight_progress_max_rate", description="Weight (dimensionless) of the weak constraint that pushes the path coordinate s to progress at the rate maxvel. Should stay small compared to the other weights", default = 0.001, required=false, minimum = 0}),
    reqs.params.scalar({name="K_progress_bounds", description="Feedback gain [1/s] of the constraints that keep the path coordinate s within [0, 1]. Larger values push s back into the range faster", default = 4, required=false, minimum = 0}),
    reqs.params.scalar({name="weight_progress_bounds", description="Weight (dimensionless) of the constraints that keep the path coordinate s within [0, 1], relative to the other constraints of the same priority", default = 20, required=false, minimum = 0}),
})
-- spl:readPoints(etasl_application_share_dir.."/scripts/etasl/motion_models/hole_in_cylinder_contour.csv"," \t,",0)

-- ========================================= PARAMETERS ===================================
maxvel    = param.get("maxvel")
maxacc    = param.get("maxacc")
csv_file_path = param.get("csv_file_path")

-- ======================================== Robot model requirements ========================================
robot = reqs.robot_model({--This function loads the robot model and checks that all required frames are available
    param.get("task_frame"), --The frame is selected as a parameter, to make the skill even more reusable
})
task_frame = robot.getFrame(param.get("task_frame"))


-- =========================== DEGREE OF ADVANCEMENT =============================================

progress_variable = Variable{context = ctx, name ='path_coordinate', vartype = 'feature', initial = 0.0}
-- =============================== INITIAL POSE ==============================

startpose = initial_value(time, task_frame)
startpos  = origin(startpose)
startrot  = rotation(startpose)

-- ========================================== Imports utils_ts ============================
-- The following is done because utils_ts is a file of the library and not of the application ROS2 package.
local script_dir = debug.getinfo(1, "S").source:match("@(.*)/")
package.path = script_dir .. "/utilities/?.lua;" .. package.path  -- Add it to package.path
local utils_ts = require("utils_ts")

-- ========================================== GENERATE ORIENTATION PROFILE ============================

R_end = startrot*rot_z(constant(param.get("rotation_z_end")))

-- eq. axis of rotation for rotation from start to end:w
diff_rot                = cached(getRotVec( inv(startrot)*R_end ))
diff_rot, angle         = utils_ts.normalize( diff_rot )
--
r_inst = angle*progress_variable

-- ========================================== GENERATE PROFILES ============================

spl        = CubicSpline(0)
csv_file = utils_ts.path_interpolate(csv_file_path)
spl:readPoints(csv_file," \t,",0)
spl:setInput(progress_variable)
-- =========================== VELOCITY PROFILE ============================================
d_time = constant(0)
mt=constant(1)

A = conditional(time-d_time,constant(1),constant(0))

sa_n = 0
sb_n = 1

sa = constant(sa_n)
sb = constant(sb_n)
s_p_mp  = utils_ts.trap_velprofile( maxvel , maxacc , constant(0.0) , constant(1.0),  progress_variable)
s_p = conditional( time-d_time , s_p_mp , constant(0) )


Constraint{
	context=ctx,
	name = "vel_prof",
	expr = progress_variable - s_p*time,
	weight = A*constant(param.get("weight_progress_profile")),
	priority = 2,
	K = constant(0)
};

Constraint{
	context=ctx,
	name = "reaching_vel_max",
	expr = progress_variable - maxvel*time,
	weight = A*constant(param.get("weight_progress_max_rate")),
	priority = 2,
	K = constant(0)
};

Constraint{
	context=ctx,
	name = "s_min",
	expr = progress_variable,
	target_lower = sa,
	weight = constant(param.get("weight_progress_bounds")),
	priority = 2,
	K = constant(param.get("K_progress_bounds"))
};

Constraint{
	context=ctx,
	name = "s_max",
	expr = progress_variable,
	target_upper = sb,
	weight = constant(param.get("weight_progress_bounds")),
	priority = 2,
	K = constant(param.get("K_progress_bounds"))
};

-- ========================= FOLLOW GENERATED POSE ========================


e_x  = getSplineOutput(spl,0)
e_y  = getSplineOutput(spl,1)
e_z  = getSplineOutput(spl,2)



p_c= startpos + vector(e_x,e_y,e_z)

targetpos = p_c
targetrot = startrot*rotVec(diff_rot, r_inst)

target    = frame(targetrot,targetpos)

Constraint{
    context = ctx,
    name    = "follow_path_rotation",
    expr    = inv(rotation(target))*rotation(task_frame),
    weight  = constant(param.get("weight_rotation")),
    K       = constant(param.get("K_rotation")),
    priority= 2
}

Constraint{
    context = ctx,
    name    = "follow_path_position",
    expr    = origin(target) - origin(task_frame),
    weight  = constant(param.get("weight_position")),
    K       = constant(param.get("K_position")),
    priority= 2
}


-- =================================== MONITOR TO FINISH THE MOTION ========================

err = (sb-progress_variable)
Monitor{
        context=ctx,
        name='finish_after_motion',
        lower=0.000,
        actionname='exit',
        expr=err
}


-- ============================== OUTPUT PORTS===================================

ctx:setOutputExpression("time",time)
ctx:setOutputExpression("x_tf",coord_x(origin(task_frame)))
ctx:setOutputExpression("y_tf",coord_y(origin(task_frame)))
ctx:setOutputExpression("z_tf",coord_z(origin(task_frame)))
ctx:setOutputExpression("progress_variable",progress_variable)