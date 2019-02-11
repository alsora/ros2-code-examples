# Simple Navigation

The Navigation stack is one of the most popular ROS packages.
It has been recently ported to ROS2, together with a lot of new features, during the Crystal release.
You can find the full source code [here](https://github.com/ros-planning/navigation2).


## Build

The recommended way for getting these packages is to install them from debian packages.

    $ sudo apt-get install -y ros-crystal-nav2-*


## Usage

The ROS2 Navigation stack contains packages for driving a mobile robot to a goal, representing a map and localizing in it.

In order to use these packages, you will need to feed them with sensor data. These can be produced by a real robot or by a Gazebo simulation.

In the following examples, we will go through how to use navigation2 in a Gazebo simulation for a Turtlebot3 robot.


### Setup a gazebo simulated environment

Install gazebo following the [instructions](../simple_gazebo).

Install the sensor nodes and the utilities for the Turtlebot3 robot.

```
$ mkdir -p ~/turtlebot3_ws/src
$ cd ~/turtlebot3_ws/src
$ git clone -b ros2 https://github.com/ROBOTIS-GIT/turtlebot3.git
$ git clone -b ros2 https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
$ git clone -b ros2 https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
$ cd ..
$ source <PATH_TO_ROS2_SDK_WS>/install/setup.sh
$ colcon build
```

Add the Turtlebot model to the Gazebo models

```
$ echo 'export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/turtlebot3_ws/src/turtlebot3_simulations/turtlebot3_gazebo/models' >> ~/.bashrc
$ source ~/.bashrc
```

Run the robot's sensor nodes

    $ ros2 launch turtlebot3_bringup burger_state_publisher.launch.py use_sim_time:=true

Note the `use_sim_time:=true` option: it's required when we are working in simulations.

Run Gazebo and load a world with our robot in it

    $ gazebo --verbose turtlebot3_ws/install/turtlebot3_gazebo/share/turtlebot3_gazebo/worlds/turtlebot3.world -s libgazebo_ros_init.so

We are passing the path to a .world file which contains the definition of an environment and the position of a robot in it.
Wait few instants as the gazebo interface is loaded and you will see everything.

Try to teleoperate the robot, to check if everything has been setup correctly.

    $ ros2 run turtlebot3_teleop teleop_keyboard


### Move and Localize on a given map

The `map_server` node provides maps to the rest of the system, using both topic and service interfaces.
The standard way of using this node consists in having a static map, defined by an image and a YAML configuration file, and loading it when the node is created.

The `world_model` node enriches this map with local and global costmaps, each of them showing which areas are reachable/safe for driving according to the specific robot shape.

The `amcl` node uses Adaptive Monte Carlo techniques in order to localize the robot on a given map, according to its sensor readings.
It's possible to set an initial estimate for the pose publishing a `geometry_msgs::msg::PoseWithCovarianceStamped` on the `initialpose` topic.

The `dwb_controller` node is a local planner action server, for the `FollowPath` action. Given a path and a current position, it will produce a command velocity.
It's based on Dynamic Window Approach.

The `navfn_planner` node is a global planner action server, for the `ComputePathToPose` action. It accepts requests containing an end pose, it gathers the current pose and then produces a corresponding plan.
It's based on Global Dyanmic Window Approach, with Dijkstra or A* as search algorithms.

The `simple_navigator` node implements the `NavigateToPose` action server. Given a goal pose, it will first call the `ComputePathToPose` action, then the `FollowPath` one.
Note that there are no fallbacks or recovery methods implemented in this node: it simply waits for completion or failure of each sub action.
It implements also a subscription to the `move_base_simple/goal` topic, where it's possible to publish a `geometry_msgs::msg::PoseStamped` denoting the desired goal.

Nodes which require access to the robot pose, the odometry reading or have to publish velocities command, create an instance of a `robot` node which instantiates the proper publisher and subscribers.

In order to run all the aforementioned nodes:

    $ ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=true
    

Then, through rviz2, first set a initial guess for the robot pose using the `2D Pose Estimate` button.
Once the initial pose has been refined, you can start navigating sending a goal with the `2D Nav Goal` button.


### More autonomy with Behavior Trees

The high-level brain of the system presented in the previous example was the `simple_navigator` node.
However it was not very smart, as it was not able to take into account any unexpected situation.

More complex behaviors can be defined using the `bt_navigator` node, which doesn't simply call the actions in a sequential order as before, but following a behavior tree.

A behavior tree can be seen as a variant of a finite state machine.
You can read more information about [finite state machines](http://robot.unipv.it/toolleeo/teaching/docs_robotics/fsm.pdf) and [behavior trees](https://github.com/BehaviorTree/BehaviorTree.CPP/blob/master/docs/BT_basics.md).

The behavior tree that the navigator should follow is specified through an XML file, as the one you can find [here](https://github.com/ros-planning/navigation2/tree/master/nav2_bt_navigator/behavior_trees).
Nodes in a behavior tree can share variables using the blackboard. This is a key/value storage shared by all the elements of a tree.
Thus everyone can set/get values from it.

The `bt_navigator` node, as the `simple_navigator` one, will implement a `NavigateToPose` action server.
The elements relevant to this task, as the goal position and the path, will be stored in the blackboard.

You can run a ROS system with all the previous nodes, but the `bt_navigator` instead of the `simple_navigator` running

    $ ros2 launch <PATH_TO_THIS_REPO>/simple_navigation/bt_navigation.launch.py use_sim_time:=true

Every time a `NavigateToPose` request is sent, the `bt_navigator` node will load a new behavior tree XML file.
You can specify which file should be loaded using a ROS2 parameter.

    $ ros2 param set /bt_navigator bt_xml_filename <PATH_TO_THIS_REPO>/simple_navigation/behavior_trees/simple_sequential.xml

This will set the parameter to the simplest behavior tree, which is equivalent to the sequence of actions performed by the `simple_navigator` node.

You can find more complex behavior trees in the [navigation2 repo](https://github.com/ros-planning/navigation2/tree/master/nav2_bt_navigator/behavior_trees).

Then you can use it as in the previous example, simply by setting the initial pose and requesting a goal.

### Build a map using SLAM

The navigation stack does not include any SLAM algorithm.
For this reason, you have to install an external one as Cartographer, which is the ROS2 recommended SLAM algorithm.

```
$ sudo apt-get install -y \
    libceres-dev \
    libcairo2-dev \
    liblua5.3-dev \
    google-mock \
    libpcl-dev \
    libboost-all-dev \
    libeigen3-dev \
    libgflags-dev \
    libgoogle-glog-dev \
    libsuitesparse-dev \
    ninja-build \
    python-sphinx

$ mkdir -p ~/cartographer_ws/src
$ HOME/cartographer_ws/src
$ git clone -b crystal https://github.com/ROBOTIS-GIT/cartographer.git
$ git clone -b crystal https://github.com/ROBOTIS-GIT/cartographer_ros.git
$ git clone https://github.com/ros2/pcl_conversions
$ cd ..
$ source <PATH_TO_ROS2_SDK_WS>/install/setup.sh
$ colcon build

```

The `cartographer_node` is a ROS2 node wrapping the Cartographer algorithm.
It will subscribe to sensor data and it will produce an incrementally updated occupancy grid on the `map` topic.

In order to run this node and rviz2 for visualization:

    $ ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=True

Then you can teleoperate the robot around, in order to build the map.

    $ ros2 run turtlebot3_teleop teleop_keyboard
