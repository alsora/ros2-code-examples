# Simple Backward Compatibility


This package is meant to be used to demonstrate how to create messages with optional fields and how to exploit them for achieving backward compatibility.

Long story short, usually when you add a new field to a message interface and then you try to send these updated messages to a subscriber which has been compiled against an old version of the message library (i.e. without the new field) a memory error will arise.


ROS2 introduces optional fields which are dynamically allocated by the subscriber.


## How to use it

Let's start by creating a workspace and compile two simple publisher and subscriber nodes.

```
mkdir -p ws_v1/src
cd ws_v1/src
ln -s <PATH_TO_THIS_REPO>/simple_backward_compatible .
cd ..
colcon build
```

The nodes created in this workspace will communicate using [this message interface](interfaces_v1/msg/Optional_fields.msg) which we will call v1.

You can immediately run the subscriber node.

```
source install/local_setup.sh
ros2 run simple_backward_compatible subscriber_main
```

Now, while the subscriber is still running, open a new terminal and create a second workspace.

```
mkdir -p ws_v2/src
cd ws_v2/src
ln -s <PATH_TO_THIS_REPO>/simple_backward_compatible .
cd ..
```

However this time, before compiling, let's make a quick change.
Open with a text editor `CMakeLists.txt` and uncomment line #5 (the one saying `#set(BUILD_V2 true)`).
Then build.

```
colcon build
```

You have just told the compiler to use a different version of the message interface, which you can find [here](interfaces_v2/msg/Optional_fields.msg) and we will call it v2.

As you can see, the new messages have some additional fields characterized by variable length arrays.
You can note from the [publisher code](src/publisher_main.cpp) that the fields of the updated message are being properly filled with values.

Now run a publisher node from this second workspace.

```
source install/local_setup.sh
ros2 run simple_backward_compatible publisher_main
```

You will note that the two nodes are communicating properly, even if compiled against two different versions of the message interfaces.

**NOTE**: if you try to run a v2 subscriber and a v1 publisher you will still get a memory error.