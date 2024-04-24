/*@license BSD-3 https://opensource.org/licenses/BSD-3-Clause
Copyright (c) 2024, Institute of Automatic Control - RWTH Aachen University
Maximilian Nitsch (m.nitsch@irt.rwth-aachen.de)
All rights reserved.
*/

#pragma once

#include <memory>
#include <random>

#include "imu_simulator_structures.h"

namespace imu_simulator {

class ImuSimulator {
 public:
  // Constructor
  explicit ImuSimulator(
      ImuSimParams accSimParams = ImuSimParams(),
      ImuSimParams gyroSimParams = ImuSimParams(),
      ImuModelEnableSettings imuModelEnableSettings = ImuModelEnableSettings(),
      Eigen::Vector3d geoPositionLlh = Eigen::Vector3d::Zero(),
      Eigen::Vector3d v_eb_n = Eigen::Vector3d::Zero(), double dt = 0.01,
      unsigned int seed = 42);

  // Destructor
  ~ImuSimulator();

  // Measurement generation functions
  Eigen::Vector3d generateAccelerationMeasurement(
      const Eigen::Vector3d& a_ib_b_true, const Eigen::Quaterniond q_b_n_true);
  Eigen::Vector3d generateGyroscopeMeasurement(
      const Eigen::Vector3d& w_nb_b_true, const Eigen::Quaterniond q_b_n_true);

  // Reset function to reset Gauss-Markov processes and constant errors
  void resetImuSimulator();

  // Getter functions
  ImuSimParams getImuSimParams(const MeasurementType& measurementType) const;
  EnvironmentalParameters getEnvironmentalParameters() const;
  Eigen::Vector3d getConstTurnOnBias(
      const MeasurementType& measurementType) const;
  ImuStochasticErrors getImuStochasticErrors(
      const MeasurementType& measurementType) const;
  Eigen::Vector3d getImuGeoPositionLlh() const;
  Eigen::Vector3d getImuGeoVelocity() const;
  Eigen::Vector3d getConstScaleFactor(
      const MeasurementType& MeasurementType) const;
  Eigen::Vector3d getConstMisAlignment(
      const MeasurementType& MeasurementType) const;
  Eigen::Matrix<double, 6, 1> getConstNonOrthogonality(
      const MeasurementType& MeasurementType) const;

  // Setter functions
  void setImuSimParameters(const ImuSimParams& accSimParams,
                           const ImuSimParams& gyroSimParams);
  void setEnvironmentalParameters(
      const EnvironmentalParameters& environmentalParameters);

  void setSimulatorSeed(const int seed);
  void setImuSampleTime(const double dt);

  void setImuGeoPositionLlh(const Eigen::Vector3d geoPositionLlh);
  void setImuGeoVelocity(const Eigen::Vector3d v_eb_n);

  void setImuEnableSettings(
      const ImuModelEnableSettings& imuModelEnableSettings);

  void setAccEnableGravity(const bool enable);
  void setAccEnableTurnOnBias(const bool enable);
  void setAccEnableScaling(const bool enable);
  void setAccEnableMisAlignment(const bool enable);
  void setAccEnableStochasticError(const bool enable);
  void setAccEnableSaturation(const bool enable);
  void setAccEnableQuantization(const bool enable);

  void setGyroEnableEarthAngularVelocity(const bool enable);
  void setGyroEnableTurnOnBias(const bool enable);
  void setGyroEnableScaling(const bool enable);
  void setGyroEnableMisAlignment(const bool enable);
  void setGyroEnableStochasticError(const bool enable);
  void setGyroEnableSaturation(const bool enable);
  void setGyroEnableQuantization(const bool enable);

  void setUseFixedRandomNumbersFlag(const bool enable);

  // Print functions
  std::stringstream printImuSimulatorParameters();

 private:
  // IMU simulation parameters
  ImuSimParams accSimParams_;
  ImuSimParams gyroSimParams_;

  // IMU Stochastic error states vectors (XYZ-axis)
  std::shared_ptr<ImuStochasticErrors> pAccStochasticErrors_;
  std::shared_ptr<ImuStochasticErrors> pGyroStochasticErrors_;

  // IMU constant errors (after random sampling)
  Eigen::Vector3d accConstTurnOnBias_;
  Eigen::Vector3d accConstScaleFactor_;
  Eigen::Matrix<double, 6, 1> accConstNonOrthogonality_;
  Eigen::Vector3d accConstMisAlignment_;

  Eigen::Vector3d gyroConstTurnOnBias_;
  Eigen::Vector3d gyroConstScaleFactor_;
  Eigen::Matrix<double, 6, 1> gyroConstNonOrthogonality_;
  Eigen::Vector3d gyroConstMisAlignment_;

  // IMU model settings (enable/disable stochastic errors, etc.)
  ImuModelEnableSettings imuModelEnableSettings_;

  // Flag indicating if constant errors have been initialized
  bool constErrorsInitialized_;

  // Environmental parameters
  EnvironmentalParameters environmentalParameters_;

  // Geodetic position (latitude, longitude, height) of body frame (IMU)
  Eigen::Vector3d geoPositionLlh_;

  // Velocity of body frame (IMU) with respect to the ECEF frame
  Eigen::Vector3d v_eb_n_;

  // Transport velocity of NED frame with respect to ECEF frame
  Eigen::Vector3d w_en_n_;

  // IMU sampling time
  double dt_;

  // Random number generator for normal distribution and uniform distribution
  std::mt19937 randomNumberGenerator_;
  std::normal_distribution<> normalDistribution_;
  std::uniform_real_distribution<> uniformDistribution_;

  // Flag indicating if fixed random numbers are be used (for testing/debugging)
  bool useFixedRandomNumbers_;

  Eigen::Vector3d calcAccelerometerGravityModel(
      const Eigen::Vector3d& a_ib_b, const Eigen::Quaterniond& q_b_n_true);

  Eigen::Vector3d calcGyroscopeEarthAngularVelocityModel(
      const Eigen::Vector3d& w_nb_b_true, const Eigen::Quaterniond& q_b_n_true);

  // WELMEC gravity model
  Eigen::Vector3d calcGravityVectorWelmec(
      const Eigen::Vector3d& geoPositionLlh);

  // Earth angular velocity model
  Eigen::Vector3d calcAngularVelocityOfEarth(
      const Eigen::Vector3d& geoPositionLlh);

  // Transport angular velocity model
  Eigen::Vector3d calcTransportAngularVelocity(
      const Eigen::Vector3d& geoPositionLlh, const Eigen::Vector3d& v_eb_n);

  // Turn-on bias model
  Eigen::Vector3d calcConstTurnOnBiasModel(
      const Eigen::Vector3d& measurement,
      const MeasurementType& measurementType);

  Eigen::Vector3d calcScaleFactorModel(const Eigen::Vector3d& measurement,
                                       const Eigen::Vector3d& scaleFactor);

  // Scale factor and misalignment model
  Eigen::Vector3d calcMisalignmentModel(
      const Eigen::Vector3d& measurement,
      const Eigen::Matrix<double, 6, 1>& nonOrthogonality,
      const Eigen::Vector3d& misAlignment);

  // Stochastic error model (colored noise)
  Eigen::Vector3d calcStochasticErrorModel(
      const Eigen::Vector3d& measurement,
      const MeasurementType& measurementType);

  // Update the stochastic error state space model (Gauß-Markov process)
  double updateDiscreteTimeEquivalentStateSpaceModel(
      std::shared_ptr<ImuStochasticErrors> pImuStochasticErrors, const double N,
      const double B, const double K, const double corrTime,
      const AxisIdentifier axisId);

  // Saturation model
  Eigen::Vector3d calcSaturationModel(const Eigen::Vector3d& measurement,
                                      const Eigen::Vector3d& measRange);

  // Quantization model
  Eigen::Vector3d calcQuantizationModel(const Eigen::Vector3d& measurement,
                                        const Eigen::Vector3d& resolution);

  // Rancom constant initialization (turn-on bias, scale factor, misalignment)
  void initializeRandomConstantErrors();

  // Random number generation functions
  double drawRandNormalDistNum();
  double drawRandUniformDistNumFromInterval(const double interval);
};

}  // namespace imu_simulator
