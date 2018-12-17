# simple_rqt

rqt2 is the ported version of [rqt](http://wiki.ros.org/rqt) to ROS2.
It's a GUI tool extremely customizable through the creation of plugins which allows to visualize the content of a ROS2 graph or to create simple nodes to interact with it.

## Build

**NOTE**: on December 13th 2018 the rqt2 packages have been added to the [ros2.repos](https://github.com/ros2/ros2/blob/master/ros2.repos) list.
Thus, if you installed ROS2 after this date, you can skip the build section.

Install some Python3 dependencies

    $ pip3 install -U --user pyside2
    $ pip3 install -U --user pydot

Create the workspace

    $ mkdir -p ~/rqt2_ws/src
    $ cp rqt2.repos ~/rqt2_ws/
    $ cd ~/rqt2_ws
    $ vcs import src --force < rqt2.repos


Install rqt dependencies specifying the path to the ROS2 SDK workspace (in the docker environment it's `~/ros2_ws`)

    $ rosdep install --from-paths src <PATH_TO_ROS2_SDK_WS>/src --ignore-src --rosdistro bouncy -y --skip-keys "console_bridge fastcdr fastrtps libopensplice67 rti-connext-dds-5.3.1 urdfdom_headers"

Build the workspace

    $ source <PATH_TO_ROS2_SDK_WS>/install/setup.sh
    $ colcon build

Source the workspace

    $ source install/local_setup.sh


## Run rqt2

Now you are ready to run rqt2.

The main entry-point is the rqt gui.

    $ ros2 run rqt_gui rqt_gui

From the gui you can run all the plugins that you have installed for rqt.
Note that all the plugins will be attached to the same gui window.

You can find a list of avilable plugin under the `Plugins` tab of the rqt gui window.

**NOTE**: If you already know that you will only need a specific plugin, you can also directly run in from the terminal instead of selecting it from the gui.

#### Topics publisher plugin

From the rqt gui window, select `Plugins/Topics/Message Publisher`.

Alternatively run `ros2 run rqt_publisher rqt_publisher`.

This plugin allows you to create publishers and use them to send messages.

You have to specify the `Topic` value, i.e. the name of the topic where you want to publish messages.
From the dropdown menu you can see a list of already active topics, otherwise you can type a new name if you want to create a new topic and start publishing on it.
Then you have to specify the `Type` of the message, here you have to select an item from the dropdown menu: if a message type is not there it means that the workspace where this message was built has not been sourced.
The last thing to specify is the `Frequency` in Hz at which we want to publish.

Once we have defined these parameters, we can create any number of new publishers clicking on the `+` icon.

After adding one or more publishers you will see them in the bottom part of the window.
By expanding a publisher line you can inspect the message currently published.
You can manually modify the `expression` field to publish your desired content.

For example:

 - Create a publisher on the topic `/my_topic`, with messages of type `std_msgs/String`.
 - Add a custom message to the publisher expression field, such as `'Hello from rqt!!'`.
 - Run a simple subscriber to receive the message with `ros2 run simple_subscriber subscriber_main`.
 - You can start publishing messages by selecting the white box at the left of the topic name.
 - Deselecting this box will cause the publisher to stop sending messages.
 - You can publish a single message by right clicking on the publisher name and selecting `Publish Selected Once`.


#### Topics and Services type browser plugin

From the rqt gui window, select `Plugins/Topics/Message Type Browser` or `Plugins/Services/Service Type Browser`.

Alternatively run `ros2 run rqt_msg rqt_msg` or `ros2 run rqt_srv rqt_srv`.

This plugin allows you to select any of the currently visible (i.e. sourced) messages or services and then recursively explore its fields.

You have to specify the package name and the message or service name.

For example, under the Message Type Browser window

 - Select as message `map_msgs/ProjectedMap` and add it to the window clicking on the `+` icon.
 - Then you can start expanding the message tree, seeing how it's composed by several submessages.


#### Image view plugin

From the rqt gui window, select `Plugins/Visualization/Image View`.

Alternatively run `ros2 run rqt_image_view rqt_image_view`.

This plugin allows you to subscribe to an existing topic of type `sensor_msgs/Image` and display the images published in it.
Moreover it allows you to do simple editings to each published image and/or republish images to a new topic.

For example:

 - Initially you should not see any available topic to select in the first dropdown menu on the left.
 - Start in a new terminal a node publishing images: run `ros2 run rqt_image_view image_publisher` which publishes on topic `/images`.
 - Go back to rqt, click the refresh button (the cycling blue arrows). Now you should be able to see and select the topic `/images` and the published content will be shown in the window.
 - Apply a rotation or a smooth scaling operation to the images by clicking in one of the corresponding empty boxes in the rqt window.
 - Specify a new topic where to republish the edited images by writing its name in the wide box located below the refresh button.
 - Start and stop republishing by clicking on the empty box at the left of the new topic name.

#### Logging console plugin

From the rqt gui window, select `Plugins/Logging/Console`.

Alternatively run `ros2 run rqt_console rqt_console`.

This plugin subscribes to `rosout` or any other specified logging topic and allows to display all the logged messages.
It contains tools for filtering out or highlighting specific messages.

Note that by default (as of December 2018), ROS2 loggers do not publish on any topic, but you have to enable the proper output handler for doing it.