#include "ukf.h"
#include "Eigen/Dense"
# include <iostream>
using Eigen::MatrixXd;
using Eigen::VectorXd;

/**
 * Initializes Unscented Kalman filter
 */
UKF::UKF() {
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  x_ = VectorXd(5);

  // initial covariance matrix
  P_ = MatrixXd(5, 5);

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 2.5;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 2.5;
  
  /**
   * DO NOT MODIFY measurement noise values below.
   * These are provided by the sensor manufacturer.
   */

  // Laser measurement noise standard deviation position1 in m
  std_laspx_ = 0.15;

  // Laser measurement noise standard deviation position2 in m
  std_laspy_ = 0.15;

  // Radar measurement noise standard deviation radius in m
  std_radr_ = 0.3;

  // Radar measurement noise standard deviation angle in rad
  std_radphi_ = 0.03;

  // Radar measurement noise standard deviation radius change in m/s
  std_radrd_ = 0.3;
  
  /**
   * End DO NOT MODIFY section for measurement noise values 
   */
  
  /**
   * TODO: Complete the initialization. See ukf.h for other member properties.
   * Hint: one or more values initialized above might be wildly off...
   */

  is_initialized_ = false;
  n_x_ = 5;
  n_aug_ = 7;
  lambda_ = 3 - n_x_;
  I_ = MatrixXd::Identity(n_x_, n_x_);

  weights_ = VectorXd(2 * n_aug_ + 1);
  weights_(0) = lambda_/(lambda_+n_aug_);
  for (int i = 1; i < 2 * n_aug_ + 1; ++i) {  
    weights_(i) = 0.5 / (n_aug_ + lambda_);
  }

  H_ = MatrixXd(2, 5);
  H_.fill(0.0);
  H_(0, 0) = 1.0;
  H_(1, 1) = 1.0;

  R_laser = MatrixXd(2, 2);
  R_laser << std_laspx_ * std_laspx_, 0.0, 0.0, std_laspy_ * std_laspy_;
  R_radar = MatrixXd(3, 3);
  R_radar <<  std_radr_* std_radr_, 0, 0,
              0, std_radphi_* std_radphi_, 0,
              0, 0, std_radrd_* std_radrd_;
}

UKF::~UKF() {}

void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  /**
   * TODO: Complete this function! Make sure you switch between lidar and radar
   * measurements.
   */
  //std::cout<<"process"<<std::endl;
  if (!is_initialized_) {
    VectorXd z = meas_package.raw_measurements_;
    if (meas_package.sensor_type_ == MeasurementPackage::SensorType::LASER) {
      x_ << z(0), z(1), 0, 0, 0;
      P_ << std_laspx_*std_laspx_, 0, 0, 0, 0,
            0, std_laspy_*std_laspy_, 0, 0, 0,
            0, 0, 1, 0, 0,
            0, 0, 0, 1, 0,
            0, 0, 0, 0, 1;
    }
    else {
      double rho = z(0), phi = z(1), rho_ = z(2);
      double x = rho * cos(phi), y = rho * sin(phi);
      x_ << x, y, rho_, phi, 0;
      P_ << std_radr_ * std_radr_, 0, 0, 0, 0,
          0, std_radr_ * std_radr_, 0, 0, 0,
          0, 0, 1, 0, 0,
          0, 0, 0, 1, 0,
          0, 0, 0, 0, 1;
    }
    time_us_ = meas_package.timestamp_;
    is_initialized_ = true;
    return;
  }
  
  
  double delta_t = (meas_package.timestamp_ - time_us_) / 1000000.0;
  time_us_ = meas_package.timestamp_;

  this->Prediction(delta_t);

  if (meas_package.sensor_type_ == MeasurementPackage::SensorType::LASER && use_laser_) {
    this->UpdateLidar(meas_package);
  }

  if (meas_package.sensor_type_ == MeasurementPackage::SensorType::RADAR && use_radar_) {
    this->UpdateRadar(meas_package);
  }

}

void UKF::Prediction(double delta_t) {
  /**
   * TODO: Complete this function! Estimate the object's location. 
   * Modify the state vector, x_. Predict sigma points, the state, 
   * and the state covariance matrix.
   */

  //Get sigma points
  //std::cout << "predict" <<std::endl;
  /*MatrixXd Xsig = MatrixXd(5, 2 * n_x_ + 1);
  MatrixXd A = P_.llt().matrixL();
  
  Xsig.col(0) = x_;
  for (int i = 0; i < n_x_; i++) {
      Xsig.col(i + 1) = x_ + sqrt(lambda_ + n_x_) * A.col(i);
      Xsig.col(i + n_x_  +1) = x_ - sqrt(lambda_ + n_x_) * A.col(i);
  }*/

  //Augment sigma points
  //std::cout << "aug" <<std::endl;
  VectorXd x_aug = VectorXd(7);
  x_aug.fill(0.0);
  MatrixXd P_aug = MatrixXd(7, 7);
  P_aug.fill(0.0);
  MatrixXd Xsig_aug = MatrixXd(n_aug_, 2 * n_aug_ + 1);

  x_aug.head(n_x_) = x_;
  P_aug.topLeftCorner(n_x_, n_x_) = P_;
  P_aug(5,5) = std_a_ * std_a_;
  P_aug(6,6) = std_yawdd_ * std_yawdd_;

  MatrixXd A_aug = P_aug.llt().matrixL();

  Xsig_aug.col(0) = x_aug;
  for (int i = 0; i < n_aug_; i++) {
      Xsig_aug.col(i + 1) = x_aug + sqrt(lambda_ + n_aug_) * A_aug.col(i);
      Xsig_aug.col(i + 1 + n_aug_) = x_aug - sqrt(lambda_ + n_aug_) * A_aug.col(i);
  }

  // Predict sigma points
  //std::cout << "pred sigma" <<std::endl;
  Xsig_pred_ = MatrixXd(n_x_, 2 * n_aug_ + 1);
  for (int i = 0; i < 2 * n_aug_  + 1; i++) {
      
      VectorXd c = Xsig_aug.col(i);
      VectorXd term1 = VectorXd(n_x_);
      VectorXd term2 = VectorXd(n_x_);;
      
      //std::cout << "0" <<std::endl;
      term2 << 0.5 * pow(delta_t, 2) * cos(c(3)) * c(5), 
      0.5 * pow(delta_t, 2) * sin(c(3)) * c(5), 
      delta_t * c(5), 
      0.5 * pow(delta_t, 2) * c(6), 
      delta_t * c(6);
      VectorXd x = c.head(n_x_);
      if (std::fabs(c(4)) < 1e-10) {
        //std::cout << "1" <<std::endl;
          term1 << c(2) * cos(c(3)) * delta_t, c(2) * sin(c(3)) * delta_t, 0, 0, 0;
          
      }
      else {
        //std::cout << "2" <<std::endl;
          term1 << c(2) * (sin(c(3) + c(4) * delta_t) - sin(c(3))) / c(4),
                   c(2) * (-cos(c(3) + c(4) * delta_t) + cos(c(3))) / c(4),
                   0, c(4) * delta_t, 0;
      }
      //std::cout << "3" <<std::endl;
      //for(int j = 0; j < n_x_; j++){
      //  Xsig_pred_(j, i) = c(j) + term1(j) + term2(j);
      //}
      Xsig_pred_.col(i) = x + term1 + term2;
  }

  //Get mean and covariance
  //std::cout << "mean & cov" <<std::endl;
  x_ = Xsig_pred_ * weights_;

  P_.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; ++i) {  
   
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    x_diff(3) = fmod(x_diff(3) + M_PI, 2 * M_PI);
    if (x_diff(3) < 0) x_diff(3) = x_diff(3) + 2 * M_PI;
    x_diff(3) = x_diff(3) - M_PI;
    P_ = P_ + weights_(i) * x_diff * x_diff.transpose();
  }
}

void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
   * TODO: Complete this function! Use lidar data to update the belief 
   * about the object's position. Modify the state vector, x_, and 
   * covariance, P_.
   * You can also calculate the lidar NIS, if desired.
   */
  VectorXd z = meas_package.raw_measurements_;

  /*if (!is_initialized_) {
    x_ << z(0), z(1), 0, 0, 0;
    P_ << std_laspx_*std_laspx_, 0, 0, 0, 0,
          0, std_laspy_*std_laspy_, 0, 0, 0,
          0, 0, 1, 0, 0,
          0, 0, 0, 1, 0,
          0, 0, 0, 0, 1;
    time_us_ = meas_package.timestamp_;
    is_initialized_ = true;
    return;
  }*/
  
  //std::cout << "lidar" <<std::endl;
  MatrixXd S = H_ * P_ * H_.transpose() + R_laser;
  MatrixXd K = P_ * H_.transpose() * S.inverse();
  x_ = x_ + K * (z - H_ * x_);
  P_ = (I_ - K * H_) * P_;
}

void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
   * TODO: Complete this function! Use radar data to update the belief 
   * about the object's position. Modify the state vector, x_, and 
   * covariance, P_.
   * You can also calculate the radar NIS, if desired.
   */

  VectorXd z = meas_package.raw_measurements_;
  MatrixXd Zsig = MatrixXd(3, 2 * n_aug_ + 1);

  VectorXd z_pred = VectorXd(3);
  MatrixXd S = MatrixXd(3,3);

  // transform sigma points into measurement space
  for (int i = 0; i < 2 * n_aug_ + 1; ++i) {  // 2n+1 simga points
    // extract values for better readability
    double p_x = Xsig_pred_(0,i);
    double p_y = Xsig_pred_(1,i);
    double v  = Xsig_pred_(2,i);
    double yaw = Xsig_pred_(3,i);

    double v1 = cos(yaw)*v;
    double v2 = sin(yaw)*v;

    // measurement model
    Zsig(0,i) = sqrt(p_x*p_x + p_y*p_y);                       // r
    Zsig(1,i) = atan2(p_y,p_x);                                // phi
    Zsig(2,i) = (p_x*v1 + p_y*v2) / sqrt(p_x*p_x + p_y*p_y);   // r_dot
  }

  // mean predicted measurement
  z_pred = Zsig * weights_;

  S = (Zsig.array().colwise() - z_pred.array());
  S = S * weights_.asDiagonal() * S.transpose() + R_radar;
  MatrixXd Tc = MatrixXd(n_x_, 3);
  Tc = (Xsig_pred_.array().colwise() - x_.array()).matrix() *
       weights_.asDiagonal() * (Zsig.array().colwise() - z_pred.array()).matrix().transpose();

  // calculate Kalman gain K;
  MatrixXd K = Tc * S.inverse();
  // update state mean and covariance matrix
  VectorXd z_diff = z - z_pred;

  // update state mean and covariance matrix
  x_ = x_ + K * z_diff;
  P_ = P_ - K * S * K.transpose();

}