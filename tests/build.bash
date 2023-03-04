#!/bin/bash

#g++ -pthread rs-pose-predict.cpp -O3 -I/usr/include/librealsense2 -L/usr/lib -lrealsense2 -o pose-predict.x

#g++ -pthread navigate-to-point.cpp -O3 -I/usr/include/librealsense2 -L/usr/lib -lrealsense2 -o navigate-to-point.x

# t265-test 
g++ -pthread t265-test.cpp -O3 -I/usr/include/librealsense2 -L/usr/lib  -lrealsense2 -o t265-test.x -I/home/ieeefiu/Documents/perrito/pose-predict
