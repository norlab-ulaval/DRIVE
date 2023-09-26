#!/bin/bash

(ros2 launch drive drive.launch.xml)
pid=$!
printf $pid
