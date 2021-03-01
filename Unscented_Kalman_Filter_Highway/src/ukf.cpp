#include "ukf.h"
#include "Eigen/Dense"

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
  std_a_ = 30;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 30;
  
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

  n_x_ = 5; // State dimension
  n_aug_ = 7; // Augmented state dimension
  n_sig_ = 2 * n_aug_ + 1; // Number of sigma points
  lambda_ = 3 - n_aug_; // Sigma point spreading parameter
  Xsig_pred_ = MatrixXd::Zero(n_x_, n_sig_); // predicted sigma points matrix

  weights_ = VectorXd(n_sig_); // Weights of sigma points
  weights_(0) = 1 / (1 + n_aug_ / lambda_);
  for (int i = 1; i < 2 * n_aug_ + 1; ++i) {
    weights_(i) = 0.5 / (n_aug_ + lambda_);
  }

  // measurement noise covariance matrix
  R_radar_ = MatrixXd::Zero(3, 3);
  R_radar_(0, 0) = std_radr_ * std_radr_;
  R_radar_(1, 1) = std_radphi_ * std_radphi_;
  R_radar_(2, 2) = std_radrd_ * std_radrd_;

  R_laser_ = MatrixXd::Zero(2, 2);
  R_laser_(0, 0) = std_laspx_ * std_laspx_;
  R_laser_(1, 1) = std_laspy_ * std_laspy_;

  prev_timestamp_ = 0;

  is_initialized_ = false;
}

UKF::~UKF() {}

void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  if ((!use_laser_ && (meas_package.sensor_type_ == MeasurementPackage::LASER)) ||
      (!use_radar_ && meas_package.sensor_type_ == MeasurementPackage::RADAR)) {
    return;
  }

  if (!is_initialized_) {
    if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
      float ro = static_cast<float>(meas_package.raw_measurements_(0));
      float phi = static_cast<float>(meas_package.raw_measurements_(1));
      x_ << ro * std::cos(phi), ro * std::sin(phi), 0., 0., 0.;
    }
    else if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
      x_ << meas_package.raw_measurements_(0), meas_package.raw_measurements_(1), 0., 0., 0.;
    }

    prev_timestamp_ = meas_package.timestamp_;
    is_initialized_ = true;
    return;
  }

  /***********************************************************
   * Prediction
   ***********************************************************/

  float delta_t = static_cast<double>(meas_package.timestamp_ - prev_timestamp_);
  Prediction(delta_t);

  if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
    UpdateLidar(meas_package);
  }
  else if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
    UpdateRadar(meas_package);
  }
}

void UKF::Prediction(double delta_t) {
  /**
   * Predict sigma points, the state, and the state covariance matrix.
   */
  Eigen::MatrixXd Xsig_aug(n_aug_, n_sig_);
  AugmentedSigmaPoints(Xsig_aug);
  SigmaPointPrediction(Xsig_aug, delta_t);
  PredictMeanAndCovariance();
}

void UKF::UpdateLidar(MeasurementPackage meas_package) {
  int n_z = 2;
  // Sigma points matrix in measurement space
  Eigen::MatrixXd Zsig(n_z, n_sig_);
  for (int i = 0; i < n_sig_; i++) {
    Zsig(0, i) = Xsig_pred_(0, i);
    Zsig(1, i) = Xsig_pred_(1, i);
  }
  
  UpdateMeasurement(meas_package, Zsig, n_z);
}

void UKF::UpdateRadar(MeasurementPackage meas_package) {
  int n_z = 3;

  // Sigma points matrix in measurement space
  Eigen::MatrixXd Zsig(n_z, 2 * n_aug_ + 1);

  for (int i = 0; i < 2 * n_aug_ + 1; ++i) {
    double p_x = Xsig_pred_(0, i);
    double p_y = Xsig_pred_(1, i);
    double v = Xsig_pred_(2, i);
    double yaw = Xsig_pred_(3, i);

    double v1 = v * cos(yaw);
    double v2 = v * sin(yaw);

    Zsig(0, i) = std::sqrt(p_x * p_x + p_y * p_y);
    Zsig(1, i) = std::atan2(p_y, p_x);
    Zsig(2, i) = (p_x * v1 + p_y * v2) / std::sqrt(p_x * p_x + p_y * p_y);
  }

  UpdateMeasurement(meas_package, Zsig, n_z);
}

void UKF::AugmentedSigmaPoints(Eigen::MatrixXd& Xsig_aug) {
  Eigen::VectorXd x_aug = Eigen::VectorXd::Zero(n_aug_);
  Eigen::MatrixXd P_aug = Eigen::MatrixXd::Zero(n_aug_, n_aug_);

  x_aug.head(n_x_) = x_;
  P_aug.topLeftCorner(n_x_, n_x_) = P_;
  P_aug(5, 5) = std_a_ * std_a_;
  P_aug(6, 6) = std_yawdd_ * std_yawdd_;

  Eigen::MatrixXd L = P_aug.llt().matrixL();
  Xsig_aug.col(0) = x_aug;

  for (int i = 0; i < n_aug_; ++i) {
    Xsig_aug.col(i + 1) = x_aug + std::sqrt(lambda_ + n_aug_) * L.col(i);
    Xsig_aug.col(i + 1 + n_aug_) = x_aug - std::sqrt(lambda_ + n_aug_) * L.col(i);
  }
}

void UKF::SigmaPointPrediction(const Eigen::MatrixXd& Xsig_aug, double delta_t) {
  for (int i = 0; i< 2 * n_aug_ + 1; ++i) {
    // extract values for better readability
    double p_x = Xsig_aug(0,i);
    double p_y = Xsig_aug(1,i);
    double v = Xsig_aug(2,i);
    double yaw = Xsig_aug(3,i);
    double yawd = Xsig_aug(4,i);
    double nu_a = Xsig_aug(5,i);
    double nu_yawdd = Xsig_aug(6,i);

    // predicted state values
    double px_p, py_p;

    // avoid division by zero
    if (fabs(yawd) > 0.001) {
        px_p = p_x + v / yawd * ( sin (yaw + yawd * delta_t) - sin(yaw));
        py_p = p_y + v / yawd * ( cos(yaw) - cos(yaw + yawd * delta_t) );
    } else {
        px_p = p_x + v * delta_t * cos(yaw);
        py_p = p_y + v * delta_t * sin(yaw);
    }

    double v_p = v;
    double yaw_p = yaw + yawd * delta_t;
    double yawd_p = yawd;

    // add noise
    px_p = px_p + 0.5 * nu_a * delta_t * delta_t * cos(yaw);
    py_p = py_p + 0.5 * nu_a * delta_t * delta_t * sin(yaw);
    v_p = v_p + nu_a * delta_t;

    yaw_p = yaw_p + 0.5 * nu_yawdd * delta_t * delta_t;
    yawd_p = yawd_p + nu_yawdd * delta_t;

    // write predicted sigma point into right column
    Xsig_pred_(0,i) = px_p;
    Xsig_pred_(1,i) = py_p;
    Xsig_pred_(2,i) = v_p;
    Xsig_pred_(3,i) = yaw_p;
    Xsig_pred_(4,i) = yawd_p;
  }
}

void UKF::PredictMeanAndCovariance() {
  Eigen::VectorXd x = Eigen::VectorXd::Zero(n_x_);
  for (int i = 0; i < 2 * n_aug_ + 1; ++i) {
    x += weights_(i) * Xsig_pred_.col(i);
  }

  Eigen::MatrixXd P = Eigen::MatrixXd::Zero(n_x_, n_x_);
  for (int i = 0; i < 2 * n_aug_ + 1; ++i) {  // iterate over sigma points
    Eigen::VectorXd x_diff = Xsig_pred_.col(i) - x;
    NormalizeAngle(x_diff(3));
    P += weights_(i) * x_diff * x_diff.transpose();
  }

  x_ = x;
  P_ = P;
}

void UKF::UpdateMeasurement(MeasurementPackage& meas_package, Eigen::MatrixXd& Zsig, int n_z) {
  Eigen::VectorXd z_pred = Eigen::VectorXd::Zero(n_z);  // Mean predicted measurement
  Eigen::MatrixXd S = Eigen::VectorXd::Zero(n_z, n_z);  // Measurement covariance matrix

  for (int i = 0; i < 2 * n_aug_ + 1; ++i) {
    z_pred += weights_(i) * Zsig.col(i);
  }

  for (int i = 0; i < 2 * n_aug_ + 1; ++i) {
    Eigen::VectorXd z_diff = Zsig.col(i) - z_pred;

    if(meas_package.sensor_type_ == MeasurementPackage::RADAR) {
      NormalizeAngle(z_diff(1));
    }

    S += weights_(i) * z_diff * z_diff.transpose();
  }

  Eigen::MatrixXd R(n_z, n_z);
  Eigen::VectorXd z(n_z);
  if (meas_package.sensor_type_ == MeasurementPackage::LASER) {
    z << meas_package.raw_measurements_[0], meas_package.raw_measurements_[1];
    R << std_laspx_ * std_laspx_, 0, 0, std_laspy_ * std_laspy_;
  } else if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
    z << meas_package.raw_measurements_[0], meas_package.raw_measurements_[1], meas_package.raw_measurements_[2];
    R << std_radr_ * std_radr_, 0, 0, 0, std_radphi_ * std_radphi_, 0, 0, 0, std_radrd_ * std_radrd_;
  }
  S = S + R;

  Eigen::MatrixXd Tc = Eigen::MatrixXd::Zero(n_x_, n_z);
  for (int i = 0; i < 2 * n_aug_ + 1; ++i) {
    // residual
    Eigen::VectorXd z_diff = Zsig.col(i) - z_pred;

    if(meas_package.sensor_type_ == MeasurementPackage::RADAR) {
      NormalizeAngle(z_diff(1));
    }

    Eigen::VectorXd x_diff = Xsig_pred_.col(i) - x_;
    NormalizeAngle(x_diff(3));

    Tc += weights_(i) * x_diff * z_diff.transpose();
  }

  // Kalman gain K;
  Eigen::MatrixXd Si = S.inverse();
  Eigen::MatrixXd K = Tc * Si;

  // Residual
  Eigen::VectorXd z_diff = z - z_pred;

  // Angle normalization
  if(meas_package.sensor_type_ == MeasurementPackage::RADAR) {
    NormalizeAngle(z_diff(1));
  }

  x_ += K * z_diff;
  P_ -= K * S * K.transpose();

  // if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
  //   nis_radar_ = z_diff.transpose() * Si * z_diff;
  // } else if (meas_package.sensor_type_ == MeasurementPackage::LASER){
  //   nis_lidar_ = z_diff.transpose() * Si * z_diff;
  // }
}
