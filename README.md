# **Extended Kalman Filter Implementation Project**

[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)


## Overview

This repository is for the project of **Udacity Nanodegree - Self-driving Car Engineer : Extended Kalman Filter Proejct**.  It is forked from https://github.com/udacity/CarND-Extended-Kalman-Filter-Project).  


The goals / steps of this project are the following:
* Understand how EKF(Extended Kalman Filter) works in object detection
* Implement EKF in C++


## Environment Setting
I used Bash on Windows 10 for code complie.  

- C++ compile dependency / run `install-ubuntu.sh`
~~~sh
sudo apt-get update
sudo apt-get install git libuv1-dev libssl-dev gcc g++ cmake make
git clone https://github.com/uWebSockets/uWebSockets 
cd uWebSockets
git checkout e94b6e1
mkdir build
cd build
cmake ..
make 
sudo make install
cd ../..
sudo ln -s /usr/lib64/libuWS.so /usr/lib/libuWS.so
sudo rm -r uWebSockets
~~~

- code compile
~~~sh
cd CarND-Extended-Kalman-Filter-Project
cd build
cmake ..
make
./ExtendedKF
~~~

## File Structure
- `main.cpp` - communicates with the Simulator receiving data measurements, calls a function to run the Kalman filter, calls a function to calculate RMSE
- `FusionEKF.cpp` - initializes the filter, calls the predict function, calls the update function
- `kalman_filter.cpp` - defines the predict function, the update function for lidar, and the update function for radar
- `tools.cpp` - function to calculate RMSE, the Jacobian matrix and value correction for radar.
