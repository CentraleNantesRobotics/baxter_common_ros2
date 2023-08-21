baxter_common_ros2
==================

URDF, meshes, and custom messages describing the Baxter Research Robot from Rethink Robotics

All packages have been ported to ROS 2. Classical use of Baxter has been heavily tested (joint command, image topics)

baxter_bridge
-------------

The baxter_bridge package exposes a custom ros1<->ros2 bridge, precompiled for all Baxter's messages and specific topics.
It embeds a ROS 2 robot_state_publisher.
It avoids having to recompile ros1_bridge which can be quite long.
Running the bridge with '-s' forces it to forward all (many, many) topics, otherwise only a few are automatically forwarded and the others are forwarded dynamically when suitable.
When run from multiple computers (e.g. students doing a lab on Baxter), all bridges synchronize by default and only allow one user to publish a given topic at the same time. Set the rosparam 'allow_multiple' to True in order to allow several computers to publish on the same topic.

The ROS 1 counterpart can be installed from this repository: https://github.com/CentraleNantesRobotics/baxter_legacy



Code & Tickets
--------------

+-----------------+----------------------------------------------------------------+
| Documentation   | http://sdk.rethinkrobotics.com/wiki                            |
+-----------------+----------------------------------------------------------------+
| Issues          | https://github.com/RethinkRobotics/baxter_common/issues        |
+-----------------+----------------------------------------------------------------+
| Contributions   | http://sdk.rethinkrobotics.com/wiki/Contributions              |
+-----------------+----------------------------------------------------------------+

baxter_common Repository Overview
---------------------------------

::

     .
     |
     +-- baxter_common/           baxter_common metapackage
     |
     +-- baxter_description/      urdf and meshes describing baxter
     |   +-- urdf/
     |   +-- meshes/
     |
     +-- baxter_core_msgs/        messages and services for communication with baxter
     |   +-- msgs/
     |   +-- srvs/
     |
     +-- baxter_maintenance_msgs/ messages for baxter maintenance routines
     |   +-- msgs/
     |
     +-- rethink_ee_description/  urdf and meshes describing end effectors
     |   +-- urdf/
     |   +-- meshes/


Other Baxter Repositories
-------------------------

+------------------+-----------------------------------------------------+
| baxter           | https://github.com/RethinkRobotics/baxter           |
+------------------+-----------------------------------------------------+
| baxter_interface | https://github.com/RethinkRobotics/baxter_interface |
+------------------+-----------------------------------------------------+
| baxter_tools     | https://github.com/RethinkRobotics/baxter_tools     |
+------------------+-----------------------------------------------------+
| baxter_examples  | https://github.com/RethinkRobotics/baxter_examples  |
+------------------+-----------------------------------------------------+

Latest Release Information
--------------------------

http://sdk.rethinkrobotics.com/wiki/Release-Changes
