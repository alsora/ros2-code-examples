
#!/usr/bin/env bash


ros2 security create_key $HOME/simple_keys simple_publisher
ros2 security create_key $HOME/simple_keys simple_subscriber
ros2 security create_key $HOME/simple_keys additional_subscriber