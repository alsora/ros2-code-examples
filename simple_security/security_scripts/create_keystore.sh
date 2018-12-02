
#!/usr/bin/env bash


rm -rfv $HOME/simple_keys

ros2 security create_keystore $HOME/simple_keys
