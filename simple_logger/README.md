
# Simple Logger


This package allows to see how it is possible to use the ROS2 C++ logger API and its features.

## Usage

You can run the example with:

    ros2 run simple_logger logger_main


The example will create a node, which is automatically associated to a logger with the same name.

The logger is used to log messages with different levels. The default verbosity level is `INFO` for all loggers.

It is possible to change the verbosity level of each logger at runtime.

Moreover, we show how to create a logger that is not connected to any node.


#### ROSOUT TOPIC

A new feature introduced in January 2019, is the possibility of publishing all the logs to a `rosout` topic.
You can try it by buildin the master branch of ROS2 or any distribution successive to Crystal.

This feature is automatically enabled.
You can disable it through a command line argument

    ros2 run simple_logger logger_main __log_disable_rosout:=true
