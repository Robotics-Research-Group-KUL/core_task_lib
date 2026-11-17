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
Keeps the robot in an idle state: no constraints are imposed, so the commanded joint velocities are zero and the robot stays still, while the controller keeps running (e.g. so that joint states and outputs keep being published).
The robot model must contain the frame "tcp_frame". Outputs: position (x_tf, y_tf, z_tf) [m] and roll/pitch/yaw angles (roll_tf, pitch_tf, yaw_tf) [rad] of tcp_frame w.r.t. the robot base frame (root frame of the robot model).
It has no parameters, no input channels and no exit monitor: it runs until it is stopped externally (e.g. by the skill/state machine).]]

param = reqs.parameters(task_description,{

})


robot = reqs.robot_model({
    "tcp_frame",
    -- "forearm",
    --Add all frames that are required by the task specification
}) 

task_frame = robot.getFrame("tcp_frame")


ctx:setOutputExpression("x_tf",coord_x(origin(task_frame)))
ctx:setOutputExpression("y_tf",coord_y(origin(task_frame)))
ctx:setOutputExpression("z_tf",coord_z(origin(task_frame)))

roll_tf,pitch_tf,yaw_tf = getRPY(rotation(task_frame))
ctx:setOutputExpression("roll_tf",roll_tf)
ctx:setOutputExpression("pitch_tf",pitch_tf)
ctx:setOutputExpression("yaw_tf",yaw_tf)




