# simple_bag

**NOTE**: using `rosbag2` requires at least the ROS2 Crystal Clemmys release.

The rosbag2 package provides a command line tool for working with bagfile as well as code APIs for reading/writing bags.
A bagfile is a recorded stream of messages on specified topics.

## Build

Create the workspace

    $ mkdir -p ~/rosbag_ws/src
    $ cd ~/rosbag_ws/src
    $ git clone https://github.com/ros2/rosbag2.git
    $ cd ..

Build the packages

    $ source <PATH_TO_ROS2_SDK_WS>/install/setup.sh
    $ colcon build

Source the workspace

    $ source install/local_setup.sh


## Usage

After you have built the workspace it's possible to access to the rosbag2 features from the ROS2 command line interface.

    $ ros2 bag --help


#### Recording

In order to record messages you can run:

    $ ros2 bag record -a

This command will keep looking for all available topics. As soon as a new topic is discovered, rosbag will subscribe to it and start recording its content.
Note that the `-a` argument specifies that we want to record all the topics.

To stop recording, just kill the rosbag process with `Ctrl + c`.

The bagfile will be stored in a timestamped folder in the current directory.
However you can also specify a path:

    $ ros2 bag record -a -o /where/to/save/bagfile

You may want to not include all the available topics in the bagfile.
You can specify the name of the topics of interest as a positional argument, instead of using the `-a` argument.

    $ ros2 bag record topic_1 topic_2

Note that as soon as all the request topics are discovered by rosbag, it will stop looking for topics.

Example:

 - Start rosbag2 recording with `ros2 bag record -a -o my_simple_bag`.
 - Start a simple publisher node with `ros2 run simple_publisher publisher_main`.
 - Wait for some messages to be published and then kill the rosbag with `Ctrl + c`.


#### Playback

Once you have created a bagfile, it's possible to play it back.

    $ ros2 bag play /path/to/saved/bagfile

Note that messages will start immediately to be published after you run this command.
Moreover currently it's not possible to pause the bag.

Example:

 - Create a bagfile following the example in the previous section.
 - Start a simple subscriber node with `ros2 run simple_subscriber subscriber_main`.
 - Start rosbag2 playback with `ros2 bag play my_simple_bag`
