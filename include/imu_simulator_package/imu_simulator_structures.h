/*@license BSD-3 https://opensource.org/licenses/BSD-3-Clause
Copyright (c) 2023, Institute of Automatic Control - RWTH Aachen University
Maximilian Nitsch (m.nitsch@irt.rwth-aachen.de)
All rights reserved.
*/

#pragma once

#include <Eigen/Dense>

namespace imu_simulator {

struct ImuStochasticErrors {
  Eigen::Vector3d z_N;  // Velocity/angular random walk
  Eigen::Vector3d z_B;  // Bias instability
  Eigen::Vector3d z_K;  // Accel./rate random walk
};

struct ImuSimParams {
  Eigen::Vector3d N;  // Vel./ang. random walk (m/s^(3/2))/(rad/s^(1/2))
  Eigen::Vector3d B;  // Bias instability (m/s^2)/(rad/s)
  Eigen::Vector3d K;  // Accel./rate random walk (m/s^(5/2)))/(rad/s^(3/2)))
  Eigen::Vector3d corrTime;            // Correlation time (s)
  Eigen::Vector3d intervalTurnOnBias;  // Interval turn-on bias (m/s^2)/(rad/s)
  Eigen::Vector3d measRange;           // Measurement range (m/s^2)/(rad/s)
  Eigen::Vector3d intervalMisAlignment;  // Interval misalignment (mrad)
  Eigen::Matrix<double, 6, 1>
      intervalNonOrthogonality;  // Interval nonOrthogonality (mrad) //NOLINT
  Eigen::Vector3d intervalScaleFactor;  // Interval scale factor (%)
  Eigen::Vector3d resolution;  // Measurement resolution (m/s^2/LSB)/(rad/s/LSB)
};

struct ModelEnableSettings {
  bool enableLocalRefVec;   // on/off gravity (acc)/ Earth angular vel. (gyro)
  bool enableTurnOnBias;    // on/off constant turn-on bias given an interval
  bool enableScaling;       // on/off scaling
  bool enableMisAlignment;  // on/off misalignment
  bool enableStochasticError;  // on/off colored noise (stochastic error)
  bool enableSaturation;       // on/off saturation given a measurement range
  bool enableQuantization;     // on/off quantization given a resolution
};

struct ImuModelEnableSettings {
  ModelEnableSettings acc;
  ModelEnableSettings gyro;
};

struct EnvironmentalParameters {
  Eigen::Vector3d g_n;     // Local gravity vector (m/s²)
  Eigen::Vector3d w_ie_n;  // Local angular velocity of Earth (rad/s)
};

// Identifier for measurement axis XYZ
enum AxisIdentifier : int { axisX = 0, axisY = 1, axisZ = 2 };

// Identifier for measurement type (accelerometer or gyroscope)
enum MeasurementType { acc, gyro };

}  // namespace imu_simulator
