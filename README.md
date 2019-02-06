# ros2-code-examples

The following tutorials and examples are updated for **ROS 2 Crystal Clemmys** (Released in December 2018).

This repository also contains some `Dockerfile` for running different ROS2-based environments.
You can find them [here](docker).

## Requirements

 - [ROS2 Crystal](https://index.ros.org/doc/ros2/Installation/)


**Note**: you can still use `ROS2 Bouncy Bolson`, but you will not be able to compile the `simple_actions` package or to reproduce the examples of `simple_navigation`, `simple_bag`, `simple_gazebo` and `simple_rqt`.


## Building the examples

Source your ROS2 SDK, then create a workspace, add this repository to its sources and build the packages.

    $ source <PATH_TO_ROS2_SDK_WS>/install/setup.sh
    $ mkdir -p ws/src
    $ cd ws/src
    $ git clone https://github.com/alsora/ros2-code-examples
    $ cd ..
    $ colcon build
    $ source install/local_setup.sh

For more detailed build instructions look [here](build_ros2_packages.md)


## Running the examples

For each of the following tests, run the each command in a separate terminal window (remember that the ROS2 SDK has to be sourced in every terminal window!)


#### Publisher/Subscriber
```
    $ ros2 run simple_publisher publisher_main
```
```
    $ ros2 run simple_subscriber subscriber_main
```
One process publishes messages and the other subscribes to that topic.


#### Service/Client
```
    $ ros2 run simple_service service_main
```
```
    $ ros2 run simple_client client_main
```

One process provides a service and the other requests it.

Running  `simple_client/timer_main` contains an alternative implementation for the ROS2 client node.

#### Actions Service/Client
```
    $ ros2 run simple_action service_main
```
```
    $ ros2 run simple_action client_main
```

One process provides an action service and the other requests it.
The server node can reject the goal according to the request parameters.
The client node can cancel the action if it's taking too much time.

#### Logger utils
```
    $ ros2 run simple_logger logger_main
```

Log some messages and change the verbosity level at runtime.


#### Multithread systems
```
    $ ros2 run simple_multithread multithread_main
```

Run multiple nodes in separate threads from the same process.


#### Time Synchronization message filters
```
    $ ros2 run simple_time_sync publisher
```
```
    $ ros2 run simple_time_sync time_sync_main
```
One process publishes messages on multiple topics. Sometimes it publishes only on one topic, sometimes on all with the same timestamp and sometimes on all but with slightly different timestamps.
The other process create an approximate and an exact time subscribers.


#### Parameter servers
```
    $ ros2 run simple_parameter parameter_server_main
```
```
    $ ros2 run simple_parameter parameter_client_main
```

One process sets its own parameters. The other reads them.

#### ROS2 Navigation stack

[**README**](simple_navigation)

Autonomously move a mobile robot in the environment, using a real system or a simulation.


#### rosbag2

[**README**](simple_bag)

Record messages from topic to a file and playback them whenever you want.


#### rqt2

[**README**](simple_rqt)

GUI for inspection and interaction of a ROS2 graph.


#### gazebo

[**README**](simple_gazebo)

ROS2 integration of the 3D simulation and visualization tool Gazebo.


#### Backward compatibility

[**README**](simple_backward_compatible)

Allow nodes compiled against different versions of a message interface to communicate.


#### Secure ROS2

[**README**](simple_security)

Enable security options for your ROS2 system: nodes authentication, messages encryption and access control.


## ROS2 CLI (command line interface)

Note that these commands comes from a Python package. So if you have disabled Python support (i.e. when cross-compiling) they will not be available.

 - See a list of available commands

        $ ros2 --help

 - Print a list of <package_name> <executable_name>

        $ ros2 pkg executabels

 - Run a ROS2 node

        $ ros2 run <package_name> <executable_name>
     
 - List running ROS2 nodes
 
        $ ros2 node list

 - List visible topic names (a topic is visible if at least 1 node is publishing or subscribing to it)

        $ ros2 topic list

 - Echo what's published on a topic

        $ ros2 topic echo <topic_name>

 - Publish a message to a topic (message_content written as valid YAML)

        $ ros2 topic pub <topic_name> <message_type> <message_content>

 - Make a service request (request_content written as valid YAML)

        $ ros2 service call <service_name> <request_message_type> <request_content>
        

