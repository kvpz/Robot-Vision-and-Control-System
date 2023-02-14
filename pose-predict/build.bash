#!/bin/bash

g++ -pthread rs-pose-predict.cpp -I/usr/include/librealsense2 -L/usr/lib -lrealsense2 -o pose-predict.x

