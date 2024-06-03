#!/bin/bash

sudo docker build -t my/ros:app .
sudo docker run -it --rm my/ros:app