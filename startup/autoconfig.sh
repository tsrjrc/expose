#!/bin/bash

echo "starting !!!!!"

sudo systemctl stop roscore.service
sudo systemctl disable roscore.service
sudo systemctl enable /home/nvidia/Desktop/startup/roscore.service
sudo systemctl start roscore.service

sudo systemctl stop apm.service
sudo systemctl disable apm.service
sudo systemctl enable /home/nvidia/Desktop/startup/apm.service
sudo systemctl start apm.service

sudo systemctl stop zed.service
sudo systemctl disable zed.service
sudo systemctl enable /home/nvidia/Desktop/startup/zed.service
sudo systemctl start zed.service

sudo systemctl stop rate.service
sudo systemctl disable rate.service
sudo systemctl enable /home/nvidia/Desktop/startup/rate.service
sudo systemctl start rate.service

sudo systemctl stop control.service
sudo systemctl disable control.service
sudo systemctl enable /home/nvidia/Desktop/startup/control.service
sudo systemctl start control.service



