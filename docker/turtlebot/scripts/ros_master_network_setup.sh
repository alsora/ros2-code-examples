#!/bin/bash


WIFI_INTERFACE=$(iw dev | awk '$1=="Interface"{print $2}')

IP_ADDRESS=$(ifconfig $WIFI_INTERFACE | grep -Eo 'inet (addr:)?([0-9]*\.){3}[0-9]*' | grep -Eo '([0-9]*\.){3}[0-9]*' | grep -v '127.0.0.1')

echo "Detected WIFI_INTERFACE: ${WIFI_INTERFACE}"
echo "Detected IP_ADDRESS: ${IP_ADDRESS}"

echo " " >> $HOME/.bashrc
echo "# ROS1 network configuration" >> $HOME/.bashrc
echo "ROS_MASTER_URI=http://$IP_ADDRESS:11311" >> $HOME/.bashrc
echo "ROS_HOSTNAME=$IP_ADDRESS" >> $HOME/.bashrc
