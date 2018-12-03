
#!/usr/bin/env bash


ros2 security create_permission $HOME/simple_keys simple_publisher policies.yaml
ros2 security create_permission $HOME/simple_keys simple_subscriber policies.yaml