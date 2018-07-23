# **Sensor Fusion using Extended Kalman Filter**
---

**In this project, I have worked on the development of Kalman Filter (KF) and Extended Kalman Filter (EKF) algorithm in C++ to fuse Lidar and Radar data for tracking object position and velocity (both in x and y directions).**

## Algorithm Architecture:
---
1. Initialize Kalman Filter and Extended Kalman Filter Matrices
2. Check if it is the first measurement ?
   * If yes, initialize state `x` and covariance matrices
   * If no, go to prediction step
3. Predeiction step
   * Compute elapsed time `dt`
   * Use `dt` to compute EKF new state transition matrix `F` and process noise covariance matrix `Q`
   * Predict the state vector `x` and state covariance matrix `P` (which indicates the uncertainty in the prediction step)
4. Based on the coming data is from Lidar or Radar sensor
   * If it is Lidar data, so set up Lidar matrices `R` and `H` and call the update function for Lidar, where `R` is the senor noise covariance matrix and `H` is the transformation matrix (function) from prediction subspace to measurement subspace.
   * If it is Radar data,
      * Linearize the the measurement function `H`
      * Set up Radar matrices `H` and `R`
   * Update state with new measurement
 5. Performace Evaluation
 

The following flow-chart summerizes the algorithm:
![ekf_flow_chart](https://i.imgur.com/nUtrxA7.png)

## Environment:
---
* Ubuntu 16.04 LTS
* Udacity Self-Driving Car Nano-Degree Term2 Simulator
* cmake >= 3.5
* make >= 4.1
* gcc/g++ >= 5.4

For setting up the environmernment and how to use the simulator, check the [CarND-Extended-Kalman-Filter-Project](https://github.com/udacity/CarND-Extended-Kalman-Filter-Project)

In the following, I will be giving some brief notes about the algorithm implementation with some code snippents without going in any deep details related to the KF and EKF science.

If you need to know more about KF, please visit the lectures from [iLectureOnline](http://www.ilectureonline.com/lectures/subject/SPECIAL%20TOPICS/26/190) and the SDCND sensor fusion module.

### Step 1 : Initialize Kalman Filter and Extended Kalman Filter Matrices
In this step, I have initialized the matrices insidethe `FusionEKF` constructor. The `FusionEKF` is the class where the ProcessMeasurement logic lives. It interfaces with the main function and calls the `kalman_filter` functions needed for predict and update steps.
 Here is the initialization of the matrices:
 ```cpp
 FusionEKF::FusionEKF() {
  is_initialized_ = false;
  previous_timestamp_ = 0;
  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);
  Hj_ = MatrixXd(3, 4);

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
              0, 0.0225;
  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
              0, 0.0009, 0,
              0, 0, 0.09;
  H_laser_ << 1, 0, 0, 0,
			  0, 1, 0, 0;
  Hj_ << 1, 1, 0, 0,
         1, 1, 0, 0,
         1, 1, 1, 1;

  ekf_.P_ = MatrixXd(4, 4);
  ekf_.P_ << 1, 0, 0, 0,
		     0, 1, 0, 0,
		     0, 0, 1000, 0,
		     0, 0, 0, 1000;
  //the initial transition matrix F_
  ekf_.F_ = MatrixXd(4, 4);
  ekf_.F_ << 1, 0, 1, 0,
             0, 1, 0, 1,
             0, 0, 1, 0,
             0, 0, 0, 1;
  ekf_.Q_ = MatrixXd(4, 4);
  previous_timestamp_ = 0;
}
 ```
#### Matrices and Variables Definition:
`R_laser` : Lidar sensor measrument noise covariance matrix.
`R_radar` : Radar sensor measurment noise covariance matrix.
> The measurment noise is represented by a Gaussian distribution with zero mean and covariane matrix R

`H_laser` : Measurment matrix (function) that tranforms the state vector from the prediction subspace to the measurements subspace.
`Hj` : Jacobian matrix used to linearize the measurment fuction of non-linear models (used for Radar sensor)
`P` : State covariance matrix
`F` : State transition matrix
`Q` : Process noise covariance matrix
> Process noise is represented by Gaussian distribution with zero mean and covariance matrix Q

### Step 2 : Initialize state `x`
When the lidar or radar data comes for the first time, we initialize the state vector `x` using this data.
It is worth to mention that radar gives data in polar coordinates in terms of radial distance (`rho`), angle (bearing, phi or `theta`), and randial velocity (`rho_dot`). As we are tracking the object postion and velocity `px`, `py`, `vx`, and `vy`. we have to convert the radar readings from polar to cartesian.

Here is the code for initializing the state. It is part of the `ProcessMeasurement()` function:
```cpp
if (!is_initialized_) {
    // first measurement
    cout << "EKF: " << endl;
    ekf_.x_ = VectorXd(4);
    ekf_.x_ << 1, 1, 1, 1;
    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */
    	float rho = measurement_pack.raw_measurements_[0];
    	float theta = measurement_pack.raw_measurements_[1];
    	float rho_dot = measurement_pack.raw_measurements_[2];
    	float px = rho * cos(theta);
    	float py = rho * sin(theta);
    	float vx = 0.0;
    	float vy = 0.0;
    	ekf_.x_ << px, py, vx, vy;
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      /**
      Initialize state.
      */
    	ekf_.x_ << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1], 0, 0;
    }
    previous_timestamp_ = measurement_pack.timestamp_;
    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }
```
### Step 3 : Predeiction step
#### Compute elapsed time `dt`
#### Use `dt` to compute EKF new state transition matrix `F` and process noise covariance matrix `Q`
Our model assumes velocity is constant between time intervals  but in reality we know that an object's velocity can change due to acceleration.The model includes this uncertainty via the process noise. This random acceleration vector is described by a Gaussian distribution with zero mean and covariance matrix `Q`
This will conclude with the state transition matrix `F` is
![f_matrix](https://i.imgur.com/r6It7G9.png) 
The process noise covariance matrix `Q` is
![q_matrix](https://i.imgur.com/4Q8mldT.png)

The following code implements the both matrices with `dt`:
```cpp
  //compute the time elapsed between the current and previous measurements
  float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;	//dt - expressed in seconds
  previous_timestamp_ = measurement_pack.timestamp_;

  ekf_.F_ << 1, 0, dt, 0,
		     0, 1, 0, dt,
		     0, 0, 1, 0,
		     0, 0, 0, 1;
  float noise_ax = 9;
  float noise_ay = 9;
  float dt_2 = dt * dt;
  float dt_3 = dt_2 * dt;
  float dt_4 = dt_3 * dt;
  ekf_.Q_ << (dt_4/4)*noise_ax  ,  0    , (dt_3/2)*noise_ax  ,  0,
		     0            ,  (dt_4/4)*noise_ay, 0      ,  (dt_3/2)*noise_ay,
		     (dt_3/2)*noise_ax  , 0 ,          dt_2*noise_ax  , 0,
		     0            , (dt_3/2)*noise_ay , 0  , dt_2*noise_ay;

  ekf_.Predict();
```
### Step 4 : Update state with new measurement
#### For Radar:
As the state vector is represented in cartesian and we need to linearize for the radar measurment using the Taylor series, so the following matrix (Jacobian matrix) is the measurement matrix for radar:
![jac_matrix](https://i.imgur.com/8ZOg0lR.png)

The following code implements the jacobian matrix:
```cpp
MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
	MatrixXd Hj(3,4);
	//recover state parameters
	float px = x_state(0);
	float py = x_state(1);
	float vx = x_state(2);
	float vy = x_state(3);

	//pre-compute a set of terms to avoid repeated calculation
	float c1 = px*px+py*py;
	float c2 = sqrt(c1);
	float c3 = (c1*c2);

	//check division by zero
	if(fabs(c1) < 0.0001){
		cout << "CalculateJacobian () - Error - Division by Zero" << endl;
		return Hj;
	}

	//compute the Jacobian matrix
	Hj << (px/c2), (py/c2), 0, 0,
		  -(py/c1), (px/c1), 0, 0,
		  py*(vx*py - vy*px)/c3, px*(px*vy - py*vx)/c3, px/c2, py/c2;

	return Hj;
}
```
Then jocbian matirx `Hj` with `R_radar` matrix are used to udpate the state:
```cpp
  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates
	  Tools tools;
	  Hj_ = tools.CalculateJacobian(ekf_.x_);
	  ekf_.H_ = Hj_;
	  ekf_.R_ = R_radar_;
	  ekf_.UpdateEKF(measurement_pack.raw_measurements_);
}
```
Here is the implementation for `UpdateEFK()`:
```cpp
void KalmanFilter::UpdateEKF(const VectorXd &z){
	float px = x_(0);
	float py = x_(1);
	float vx = x_(2);
	float vy = x_(3);

	float rho = sqrt(px * px + py * py);
	float theta = atan2(py,px);
	float rho_dot;
	if(rho == 0.0)
	{
		rho_dot  = 0.0;
	}
	else
	{
		rho_dot = (px*vx+py*vy)/rho;
	}

	VectorXd z_pred(3);
	z_pred << rho, theta, rho_dot;
	VectorXd y = z - z_pred;
	while(1)
	{
		//cout << "Angle = " << y(1) << endl;
		if(y(1) >= M_PI)
		{
			y(1) = y(1) - 2*M_PI;
		}
		else if(y(1) <= -M_PI)
		{
			y(1) = y(1) + 2*M_PI;
		}
		else
		{
			break;
		}
	}
	MatrixXd H_radar_t = H_.transpose();
	MatrixXd S = H_ * P_ * H_radar_t + R_;
	MatrixXd Si = S.inverse();
	MatrixXd PHt = P_ * H_radar_t;
	MatrixXd K = PHt * Si;
	
	//new estimate
	x_ = x_ + (K * y);
	long x_size = x_.size();
	MatrixXd I = MatrixXd::Identity(x_size, x_size);
	P_ = (I - K * H_) * P_;
}
```
#### For Lidar:
The measurement matrix `H_laser` and Sensor noise covariance matrix `R_laser` are used to update the state
```cpp
else {
    // Laser updates
	  ekf_.H_ = H_laser_;
	  ekf_.R_ = R_laser_;
	  ekf_.Update(measurement_pack.raw_measurements_);
  }
```
Here is the implementation for the `Update` function:
```cpp
void KalmanFilter::Update(const VectorXd &z){
	VectorXd z_pred = H_ * x_;
	VectorXd y = z - z_pred;
	MatrixXd H_laser_t = H_.transpose();
	MatrixXd S = H_ * P_ * H_laser_t + R_;
	MatrixXd Si = S.inverse();
	MatrixXd PHt = P_ * H_laser_t;
	MatrixXd K = PHt * Si;

	//new estimate
	x_ = x_ + (K * y);
	long x_size = x_.size();
	MatrixXd I = MatrixXd::Identity(x_size, x_size);
	P_ = (I - K * H_) * P_;
}
```

###  Step 5 : Performance Evaluation
The EKF estimation is evaluated against the ground truth of the object position and velocity using the room mean squre error (`RMSE`)to know how far the estimated result is from the true result.

The equation from the RMSE is:
![rmse](https://i.imgur.com/iVHSFNh.png)

Here is the code implementing the RMSE for performance evaluation:
```cpp
VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
	VectorXd rmse(4);
	rmse << 0,0,0,0;
	if(estimations.size() != ground_truth.size()
			|| estimations.size() == 0){
		cout << "Invalid estimation or ground_truth data" << endl;
		return rmse;
	}
	for(unsigned int i=0; i < estimations.size(); ++i){

		VectorXd residual = estimations[i] - ground_truth[i];

		//coefficient-wise multiplication
		residual = residual.array()*residual.array();
		rmse += residual;
	}
	rmse = rmse/estimations.size();
	rmse = rmse.array().sqrt();
	return rmse;
}
```

## Conclusion
---
  * The EKF is tracking the object position and velocity and fusing data between two sensors
  * The Taylor serier and Jacobian matrix solved the problem of non-linearity
  * The perfomance ouput with RMSE values : [X= 0.09, Y, 0.08, VX= 0.45, VY = 0.44]
  * You can check a video DEMO for the project [here](https://youtu.be/ZDrgi5HehoY)
