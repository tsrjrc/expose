#!/bin/bash

echo "starting !!!!!"

sudo systemctl stop roscore.service
sudo systemctl disable roscore.service

sudo systemctl stop apm.service
sudo systemctl disable apm.service

sudo systemctl stop zed.service
sudo systemctl disable zed.service

sudo systemctl stop rate.service
sudo systemctl disable rate.service

sudo systemctl stop control.service
sudo systemctl disable control.service




