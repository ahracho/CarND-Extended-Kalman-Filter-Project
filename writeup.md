# **Extended Kalman Filter Project**

The goals / steps of this project are the following:
* Understand how EKF(Extended Kalman Filter) works in object detection
* Implement EKF in C++


[//]: # (Image References)

[image1]: ./images/before_tune1.png "Before Tune 1"
[image2]: ./images/before_tune2.png "Before Tune 2"
[image3]: ./images/after_tune2.png "After Tune 1"
[image4]: ./images/after_tune2.png "After Tune 2"
[image5]: ./images/final_result.png "Final Result"
[image6]: ./images/EKF_process.png "EKF Process"

## Rubric Points
### Here I will consider the [rubric points](https://review.udacity.com/#!/rubrics/748/view) individually and describe how I addressed each point in my implementation.  

### Compiling

#### 1. Your code should compile.
I completed the project under Ubuntu bash on Windows 10. I didn't modify CMakeLists.txt and other configuration files, so follow below to compile the code.  

~~~sh
cd build
cmake ..
make
./ExtendedKF
~~~

Then, launch Term 2 simulator.

### Accuracy
#### 1. RMSE must be less than  [.11, .11, 0.52, 0.52].  
At the initial state, RMSE is [0.206212, 0.0666796, 3.10099, 2.38589]. Final result is as below.  
![image5]  


### Follows the Correct Algorithm
#### 1. Sensor Fusion algorithm follows the general processing flow as taught in the preceding lessons.
In lesson, sensor fusion algorithm is described as below.  
![image6]  

Important logics are implemented in `main.cpp`. `Line 44 onMessage()` communicates with simulator and takes data input. After parsing data into proper format, it calls `fusionEKF.ProcessMeasurement()` to process Kalman Filter Algorithm.  

In `ProcessMeasurement()`function, it processes initial measurement and `predict` / `measurement update` steps.  

After x and P value update, it calculates RMSE and send the result back to the simulator.  

#### 2. Kalman Filter algorithm handles the first measurements appropriately.
After taking measurement data, measurements are processed in `ProcessMeasurement()` function.

If it is first measurement, use it to initialize `x` value. Lidar and Radar sensor data should be turned into `x` vector using different formula. Lidar data contains px and py, so just take them into x[0] and x[1].
~~~cpp
// FusionEKF.cpp : Line 86-88
VectorXd measurements = measurement_pack.raw_measurements_;
ekf_.x_[0] = measurements[0];
ekf_.x_[1] = measurements[1];
~~~

Radar data contains rho, theta and rho_dot, so it should be converted into px, py value.  
~~~cpp
// FusionEKF.cpp : Line 80-81
ekf_.x_[0] = rho * cos(theta);
ekf_.x_[1] = rho * sin(theta);
~~~

#### 3. Kalman Filter algorithm first predicts then updates.
In `ProcessMeasurement()` function, firstly update matrix F and Q using time difference(delta_t). (`Line 114-131`). 

And then, using updated F and Q matrix, get estimated x' and P' using `KalmanFilter::Predict()`. After prediction, measurement update step is implemented in `KalmanFilter::Update()` and `KalmanFilter::UpdateEKF()`. 

#### 4. Kalman Filter can handle radar and lidar measurements.
In measurement update step, Lidar and Radar sensor follows different logic. Biggest difference is in using measurement function matrix H.  

When it comes to Lidar sensor, since it contains only px and py value, by multiplying matrix H, drop out vx, vy value. So, matrix H is formed as [1, 0, 0, 0; 0, 1, 0, 0] and multiplied by estimated vector x.

But when dealing with Radar sensor value, matrix H should be totally different because it is made up of rho, theta and rho_dot values. To compare estimated x' vector and sensor raw measurement value, x should be converted. In lectures, it is called h(x) function, and I made `KalmanFilter::CartesianToPolar()` to convert x vector to polor scale.  

~~~cpp
  float rho = sqrt(pow(px, 2) + pow(py, 2));
  float theta = atan2(py, px);
  float rho_dot = (px * vx + py * vy) / rho;
~~~

When updating matrix S, K and P, it needs different matrix H (not function h(x)). Because h(x) is not a linear function, using it doesn't satisfy Gaussian distribution. We need to find a linear function which approximates function h(x). In this way, we get new Hj matrix(Jacobian matrix) and it is implemented in `Tools::CalculateJacobian()`. In Jacobian matrix, in case of distance between px and py is zero, I set value in small value instead of 0 (`Line 47-49`).  

After implementing these logic, I got a few problems.

![image1]     ![image2]  

As seen above, at some point, estimation values show big differences with ground truth. It happens when theta value is not in between -PI and PI, so it needs to be tuned with `Tools::ThetaValueCorrection()`. I corrected theta value at two points, one with sensor raw measurement, the other with `y = z - h(x')` when doing measurement update.  

After data correction, estimations are tuned as below.  

![image3]     ![image4]  

