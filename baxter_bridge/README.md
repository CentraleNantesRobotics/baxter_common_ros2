# baxter_bridge

The `baxter_bridge` package exposes a custom ros1<->ros2 bridge, precompiled for all Baxter's messages and specific topics.

It embeds a ROS 2 robot_state_publisher and also bridges the inverse kinematics services.

It avoids having to recompile ros1_bridge which can be quite long.

By default the bridge only forwards a selection of usual topics (joint command, joint states, range sensors). The others are forwarded dynamically when suitable.
It will display on Baxter's screen who is publishing on which topic.

## Compile dependencies

To compile the ROS 1 part, at least the several ROS 1 packages have to be installed:

- roscpp
- rosconsole
- roscpp_serialization
- rostime xmlrpcpp

All ROS 1 dependencies can be installed from the `deb` files available in [baxter_legacy](https://github.com/CentraleNantesRobotics/baxter_legacy).


## Compile options

By default the robot can only be controlled in `POSITION` or `VELOCITY` mode. This allows in particular the embedded auto-collision avoidance. In practice, `JointCommand` messages using another mode (`EFFORT` or `RAW_POSITION`) will be changed into `VELOCITY`.

Set the CMake flag `SAFE_CMD` to `False` to allow forwarding all modes for `JointCommand` messages.

## ROS 1 side

When run from multiple computers (e.g. students doing a lab on Baxter), all bridges synchronize by default and only allow one user to publish a given command topic at the same time.

The synchronization is done with ROS 1 that allows all bridges to communicate. Users are identified from their Linux username. Using the same username from several computers will lead to nodes having the same name in the ROS 1 graph.

Two ROS 1 rosparams tune the synchronization behavior:

- `allow_multiple` (default False): allow several users to publish on the same topic
- `forward_diplay` (default True): if `allow_multiple` is False, uses Baxter's screen to display who is currently publishing on which topics

It also advertizes two services on the ROS 1 side, mostly to synchronize bridges together when `allow_multiple` is False.

- `/bridge_auth`: request to publish on a given topic from a given user, returns the list of current topics and their users
- `/bridge_force`: reserve a limb (left / right) for a given user. Give an empty username to cancel

## Runtime options

 - `-s` forces it to forward all (many, many) topics
 - `--server` forces the instantiation of the synchronization server. There should be only one per network. Without this option, the first `baxter_bridge` trying to connect to this server will instead instantiate it.

## Parameters

When run from multiple computers (e.g. students doing a lab on Baxter), all bridges synchronize by default and only allow one user to publish a given topic at the same time.

Set the ROS 1 rosparam `/allow_multiple` to True in order to allow several computers to publish on the same topic.

Users are identified from their Linux username. Using the same username from several computers will lead to nodes having the same name in the ROS 1 graph.

## Services

The bridge advertizes two services on the ROS 2 side, to handle locally bridged topic:
- `/bridge_open`: force forwarding of a given topic. This can be useful if you want e.g. to display a topic in RViz that is not bridged yet
- `/bridge_exists`: know if a given topic is currently forwarded, and it which direction

