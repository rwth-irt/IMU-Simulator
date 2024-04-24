#include "imu_simulator.h"

#include <iomanip>
#include <iostream>

namespace imu_simulator {

/**
 * @brief Constructor of the ImuSimulator class.
 * 
 * @param accSimParams IMU simulation parameters for the accelerometer.
 * @param gyroSimParams IMU simulation parameters for the gyroscope.
 * @param imuModelEnableSettings IMU model enable settings.
 * @param geoPositionLlh Geodetic position of the IMU (latitude, longitude, height) (rad,rad,m).
 * @param v_eb_n Geodetic velocity of the IMU (north, east, down) (m/s).
 * @param dt Sampling time (s).
 * @param seed Random number generator seed.
*/
ImuSimulator::ImuSimulator(ImuSimParams accSimParams,
                           ImuSimParams gyroSimParams,
                           ImuModelEnableSettings imuModelEnableSettings,
                           Eigen::Vector3d geoPositionLlh,
                           Eigen::Vector3d v_eb_n, double dt, unsigned int seed)
    : accSimParams_(accSimParams),
      gyroSimParams_(gyroSimParams),
      pAccStochasticErrors_(std::make_shared<ImuStochasticErrors>()),
      pGyroStochasticErrors_(std::make_shared<ImuStochasticErrors>()),
      imuModelEnableSettings_(imuModelEnableSettings),
      constErrorsInitialized_(false),
      geoPositionLlh_(geoPositionLlh),
      v_eb_n_(v_eb_n),
      w_en_n_(calcTransportAngularVelocity(geoPositionLlh, v_eb_n)),
      dt_(dt),
      randomNumberGenerator_(seed),
      normalDistribution_(0, 1),
      uniformDistribution_(0, 1) {
  // Initialize IMU stochastic error structs
  pAccStochasticErrors_->z_N = Eigen::Vector3d::Zero();
  pAccStochasticErrors_->z_B = Eigen::Vector3d::Zero();
  pAccStochasticErrors_->z_K = Eigen::Vector3d::Zero();

  pGyroStochasticErrors_->z_N = Eigen::Vector3d::Zero();
  pGyroStochasticErrors_->z_B = Eigen::Vector3d::Zero();
  pGyroStochasticErrors_->z_K = Eigen::Vector3d::Zero();

  // Initialize local gravity vector
  environmentalParameters_.g_n = calcGravityVectorWelmec(geoPositionLlh_);
  // Initialize local angular velocity vector of Earth
  environmentalParameters_.w_ie_n = calcAngularVelocityOfEarth(geoPositionLlh_);

  // Initialize IMU constant errors
  initializeRandomConstantErrors();
}

/**
 * @brief Destructor of the ImuSimulator class.
*/
ImuSimulator::~ImuSimulator() {}

/**
 * @brief Getter for the IMU simulation parameters.
 * 
 * @param measurementType Measurement type (acceleration or angular velocity).
 * 
 * @return IMU simulation parameters.
*/
ImuSimParams ImuSimulator::getImuSimParams(
    const MeasurementType& measurementType) const {
  if (measurementType == acc) {
    return accSimParams_;
  } else if (measurementType == gyro) {
    return gyroSimParams_;
  } else {
    // Invalid measurement type
    std::cerr << "getImuSimParams: Invalid measurement type!\n";
    return ImuSimParams();
  }
}

/**
* @brief Getter for the environmental parameters.
*
* @return Environmental parameters.
*/
EnvironmentalParameters ImuSimulator::getEnvironmentalParameters() const {
  return environmentalParameters_;
}

/**
* @brief Getter for the constant turn-on bias.
* 
* @param measurementType Measurement type (acceleration or angular velocity).
*
* @return Constant turn-on bias.
*/
Eigen::Vector3d ImuSimulator::getConstTurnOnBias(
    const MeasurementType& measurementType) const {
  if (measurementType == acc) {
    return accConstTurnOnBias_;
  } else if (measurementType == gyro) {
    return gyroConstTurnOnBias_;
  } else {
    // Invalid measurement type
    std::cerr << "getConstTurnOnBias: Invalid measurement type!\n";
    return Eigen::Vector3d(0.0, 0.0, 0.0);
  }
}

/**
 * @brief Getter for the IMU stochastic errors.
 * 
 * @param measurementType Measurement type (acceleration or angular velocity).
 * 
 * @return IMU stochastic errors.
*/
ImuStochasticErrors ImuSimulator::getImuStochasticErrors(
    const MeasurementType& measurementType) const {
  if (measurementType == acc) {
    return *pAccStochasticErrors_;
  } else if (measurementType == gyro) {
    return *pGyroStochasticErrors_;
  } else {
    // Invalid measurement type
    std::cerr << "getImuStochasticErrors: Invalid measurement type!\n";
    return ImuStochasticErrors();
  }
}

/**
 * @brief Getter for the geodetic position of the IMU.
 * 
 * @return Last geodetic position of the IMU (latitude, longitude, height).
*/
Eigen::Vector3d ImuSimulator::getImuGeoPositionLlh() const {
  return geoPositionLlh_;
}

/**
 * @brief Getter for the geodetic velocity of the IMU.
 * 
 * @return Last geodetic velocity of the IMU (north, east, down).
*/
Eigen::Vector3d ImuSimulator::getImuGeoVelocity() const {
  return v_eb_n_;
}

/**
* @brief Getter for the constant scale factor error.
*
* @param measurementType Measurement type (acceleration or angular velocity).
*
* @return Constant scale factor error.
*/
Eigen::Vector3d ImuSimulator::getConstScaleFactor(
    const MeasurementType& measurementType) const {
  if (measurementType == acc) {
    return accConstScaleFactor_;
  } else if (measurementType == gyro) {
    return gyroConstScaleFactor_;
  } else {
    // Invalid measurement type
    std::cerr << "getConstScaleFactor: Invalid measurement type!\n";
    return Eigen::Vector3d(0.0, 0.0, 0.0);
  }
}

/**
* @brief Getter for the constant misalignment angles.
*
* @param measurementType Measurement type (acceleration or angular velocity).
*
* @return Constant misalignment angles.
*/
Eigen::Vector3d ImuSimulator::getConstMisAlignment(
    const MeasurementType& measurementType) const {
  if (measurementType == acc) {
    return accConstMisAlignment_;
  } else if (measurementType == gyro) {
    return gyroConstMisAlignment_;
  } else {
    // Invalid measurement type
    std::cerr << "getConstMisAlignment: Invalid measurement type!\n";
    return Eigen::Vector3d(0.0, 0.0, 0.0);
  }
}

/**
* @brief Getter for the constant orthogonality error angles.
*
* @param measurementType Measurement type (acceleration or angular velocity).
*
* @return Constant orthogonality error angles.
*/
Eigen::Matrix<double, 6, 1> ImuSimulator::getConstNonOrthogonality(
    const MeasurementType& measurementType) const {
  if (measurementType == acc) {
    return accConstNonOrthogonality_;
  } else if (measurementType == gyro) {
    return gyroConstNonOrthogonality_;
  } else {
    // Invalid measurement type
    std::cerr << "getConstNonOrthogonality: Invalid measurement type!\n";
    return Eigen::Matrix<double, 6, 1>({{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}});
  }
}

/**
 * @brief Setter for the IMU simulation parameters.
 * 
 * @param accSimParams IMU simulation parameters for the accelerometer.
 * @param gyroSimParams IMU simulation parameters for the gyroscope.
*/
void ImuSimulator::setImuSimParameters(const ImuSimParams& accSimParams,
                                       const ImuSimParams& gyroSimParams) {
  accSimParams_ = accSimParams;
  gyroSimParams_ = gyroSimParams;
}

/**
 * @brief Prints the IMU simulation parameters.
 * 
 * @return String stream with the IMU simulation parameters.
*/
std::stringstream ImuSimulator::printImuSimulatorParameters() {
  // Create stringstream to store the output
  std::stringstream ss;

  ss << "***************************************************************"
        "**********************************************************************"
        "*"
     << "\n";
  ss << std::left << std::setw(50) << "Starting IMU-Simulator"
     << "\n";
  ss << "***************************************************************"
        "**********************************************************************"
        "*"
     << "\n";

  // Accelerometer simulation parameters
  ss << std::left << "Accelerometer simulation parameters:\n";
  ss << std::fixed << std::setprecision(6);

  ss << std::left << std::setw(50) << "N:" << accSimParams_.N.transpose()
     << " m/s^(3/2)\n";
  ss << std::left << std::setw(50) << "B:" << accSimParams_.B.transpose()
     << " m/s^2\n";
  ss << std::left << std::setw(50) << "K:" << accSimParams_.K.transpose()
     << " m/s^(5/2)\n";
  ss << std::left << std::setw(50)
     << "corrTime:" << accSimParams_.corrTime.transpose() << " s\n";
  ss << std::left << std::setw(50)
     << "intervalTurnOnBias:" << accSimParams_.intervalTurnOnBias.transpose()
     << " m/s^2\n";
  ss << std::left << std::setw(50)
     << "measRange:" << accSimParams_.measRange.transpose() << " m/s^2\n";
  ss << std::left << std::setw(50) << "intervalMisAlignment:"
     << accSimParams_.intervalMisAlignment.transpose() << " rad\n";
  ss << std::left << std::setw(50) << "intervalNonOrthogonality:"
     << accSimParams_.intervalNonOrthogonality.transpose() << " rad\n";
  ss << std::left << std::setw(50)
     << "intervalScaleFactor:" << accSimParams_.intervalScaleFactor.transpose()
     << " %\n";
  ss << std::left << std::setw(50)
     << "resolution:" << accSimParams_.resolution.transpose() << " m/s^2/LSB\n";

  ss << "***************************************************************"
        "**********************************************************************"
        "*"
     << "\n";

  // Gyroscope simulation parameters
  ss << std::left << "Gyroscope simulation parameters:\n";
  ss << std::left << std::setw(50) << "N:" << gyroSimParams_.N.transpose()
     << " rad/s^(1/2)\n";
  ss << std::left << std::setw(50) << "B:" << gyroSimParams_.B.transpose()
     << " rad/s\n";
  ss << std::left << std::setw(50) << "K:" << gyroSimParams_.K.transpose()
     << " rad/s^(3/2)\n";
  ss << std::left << std::setw(50)
     << "corrTime:" << gyroSimParams_.corrTime.transpose() << " s\n";
  ss << std::left << std::setw(50)
     << "intervalTurnOnBias:" << gyroSimParams_.intervalTurnOnBias.transpose()
     << " rad/s\n";
  ss << std::left << std::setw(50)
     << "measRange:" << gyroSimParams_.measRange.transpose() << " rad/s\n";
  ss << std::left << std::setw(50) << "intervalMisAlignment:"
     << gyroSimParams_.intervalMisAlignment.transpose() << " rad\n";
  ss << std::left << std::setw(50) << "intervalNonOrthogonality:"
     << gyroSimParams_.intervalNonOrthogonality.transpose() << " rad\n";
  ss << std::left << std::setw(50)
     << "intervalScaleFactor:" << gyroSimParams_.intervalScaleFactor.transpose()
     << " %\n";
  ss << std::left << std::setw(50)
     << "resolution:" << gyroSimParams_.resolution.transpose()
     << " rad/s/LSB\n";

  ss << "***************************************************************"
        "**********************************************************************"
        "*"
     << "\n";

  // IMU model enable settings
  ss << std::left << "IMU model enable settings:\n";
  ss << std::left << "Accelerometer:\n";
  ss << std::left << std::setw(50)
     << "Turn-on bias:" << imuModelEnableSettings_.acc.enableTurnOnBias << "\n";
  ss << std::left << std::setw(50)
     << "Stochastic error:" << imuModelEnableSettings_.acc.enableStochasticError
     << "\n";
  ss << std::left << std::setw(50)
     << "Scaling:" << imuModelEnableSettings_.acc.enableScaling << "\n";
  ss << std::left << std::setw(50)
     << "Misalignment:" << imuModelEnableSettings_.acc.enableMisAlignment
     << "\n";
  ss << std::left << std::setw(50)
     << "Saturation:" << imuModelEnableSettings_.acc.enableSaturation << "\n";
  ss << std::left << std::setw(50)
     << "Quantization:" << imuModelEnableSettings_.acc.enableQuantization
     << "\n";

  ss << "\n";

  ss << std::left << "Gyroscope:\n";
  ss << std::left << std::setw(50)
     << "Turn-on bias:" << imuModelEnableSettings_.gyro.enableTurnOnBias
     << "\n";
  ss << std::left << std::setw(50) << "Stochastic error:"
     << imuModelEnableSettings_.gyro.enableStochasticError << "\n";
  ss << std::left << std::setw(50)
     << "Scaling:" << imuModelEnableSettings_.gyro.enableScaling << "\n";
  ss << std::left << std::setw(50)
     << "Misalignment:" << imuModelEnableSettings_.gyro.enableMisAlignment
     << "\n";
  ss << std::left << std::setw(50)
     << "Saturation:" << imuModelEnableSettings_.gyro.enableSaturation << "\n";
  ss << std::left << std::setw(50)
     << "Quantization:" << imuModelEnableSettings_.gyro.enableQuantization
     << "\n";
  ss << "***************************************************************"
        "**********************************************************************"
        "*"
     << "\n";

  // Environmental parameters
  ss << std::left << "Environmental parameters:\n";
  ss << std::left << std::setw(50)
     << "Local gravity vector:" << environmentalParameters_.g_n.transpose()
     << " m/s²\n";
  ss << std::left << std::setw(50) << "Local angular velocity of Earth:"
     << environmentalParameters_.w_ie_n.transpose() << " rad/s\n";

  ss << "***************************************************************"
        "**********************************************************************"
        "*"
     << "\n";

  // Sampling time and frequeqncy
  ss << std::left << std::setw(50) << "Sampling time:" << std::fixed
     << std::setprecision(6) << dt_ << " s\n";
  ss << std::left << std::setw(50) << "Sampling frequency:" << std::fixed
     << std::setprecision(6) << 1.0 / dt_ << " Hz\n";

  ss << "***************************************************************"
        "**********************************************************************"
        "*"
     << "\n";

  return ss;
}

/**
* @brief Setter for the environmental parameters.
*/
void ImuSimulator::setEnvironmentalParameters(
    const EnvironmentalParameters& environmentalParameters) {
  environmentalParameters_ = environmentalParameters;
}

/**
 * @brief Setter for the IMU seed of the random number generator.
*/
void ImuSimulator::setSimulatorSeed(const int seed) {
  randomNumberGenerator_.seed(seed);
}

/**
 * @brief Setter for the IMU simulator sampling time.
*/
void ImuSimulator::setImuSampleTime(const double dt) {
  dt_ = dt;
}

/**
 * @brief Setter for the body frame (IMU) geodetic position (ECEF).
*/
void ImuSimulator::setImuGeoPositionLlh(const Eigen::Vector3d geoPositionLlh) {
  geoPositionLlh_ = geoPositionLlh;
}

/**
 * @brief Setter for the body frame (IMU) geodetic velocity (NED).
*/
void ImuSimulator::setImuGeoVelocity(const Eigen::Vector3d v_eb_n) {
  v_eb_n_ = v_eb_n;
}

/**
 * @brief Setter for the whole IMU enable settings.
 * 
 * @param imuModelEnableSettings IMU model enable settings.
*/
void ImuSimulator::setImuEnableSettings(
    const ImuModelEnableSettings& imuModelEnableSettings) {
  imuModelEnableSettings_ = imuModelEnableSettings;
}

/** 
 * @brief Setter for the accelerometer gravity model enable flag.
 * 
 * @param enable Enable/disable accelerometer gravity model.
*/
void ImuSimulator::setAccEnableGravity(const bool enable) {
  imuModelEnableSettings_.acc.enableLocalRefVec = enable;
}

/**
* @brief Setter for the accelerometer turn-on bias model enable flag.
*
* @param enable Enable/disable accelerometer turn-on bias model.
*/
void ImuSimulator::setAccEnableTurnOnBias(const bool enable) {
  imuModelEnableSettings_.acc.enableTurnOnBias = enable;
}

/**
* @brief Setter for the accelerometer scaling model enable flag.
*
* @param enable Enable/disable accelerometer scaling model.
*/
void ImuSimulator::setAccEnableScaling(const bool enable) {
  imuModelEnableSettings_.acc.enableScaling = enable;
}

/**
* @brief Setter for the accelerometer misalignment model enable flag.
*
* @param enable Enable/disable accelerometer misalignment model.
*/
void ImuSimulator::setAccEnableMisAlignment(const bool enable) {
  imuModelEnableSettings_.acc.enableMisAlignment = enable;
}

/**
* @brief Setter for the accelerometer stochastic error model enable flag.
*
* @param enable Enable/disable accelerometer stochastic error model.
*/
void ImuSimulator::setAccEnableStochasticError(const bool enable) {
  imuModelEnableSettings_.acc.enableStochasticError = enable;
}

/**
* @brief Setter for the accelerometer saturation model enable flag.
*
* @param enable Enable/disable accelerometer saturation model.
*/
void ImuSimulator::setAccEnableSaturation(const bool enable) {
  imuModelEnableSettings_.acc.enableSaturation = enable;
}

/**
* @brief Setter for the accelerometer quantization model enable flag.
*
* @param enable Enable/disable accelerometer quantization model.
*/
void ImuSimulator::setAccEnableQuantization(const bool enable) {
  imuModelEnableSettings_.acc.enableQuantization = enable;
}

/**
 * @brief Setter for the gyroscope Earth angular velocity model enable flag.
 * 
 * @param enable Enable/disable gyroscope Earth angular velocity model.
*/
void ImuSimulator::setGyroEnableEarthAngularVelocity(const bool enable) {
  imuModelEnableSettings_.gyro.enableLocalRefVec = enable;
}

/**
* @brief Setter for the gyroscope turn-on bias model enable flag.
*
* @param enable Enable/disable gyroscope turn-on bias model.
*/
void ImuSimulator::setGyroEnableTurnOnBias(const bool enable) {
  imuModelEnableSettings_.gyro.enableTurnOnBias = enable;
}

/**
* @brief Setter for the gyroscope scaling model enable flag.
*
* @param enable Enable/disable gyroscope scaling model.
*/
void ImuSimulator::setGyroEnableScaling(const bool enable) {
  imuModelEnableSettings_.gyro.enableScaling = enable;
}

/**
* @brief Setter for the gyroscope misalignment model enable flag.
*
* @param enable Enable/disable gyroscope misalignment model.
*/
void ImuSimulator::setGyroEnableMisAlignment(const bool enable) {
  imuModelEnableSettings_.gyro.enableMisAlignment = enable;
}

/**
* @brief Setter for the gyroscope stochastic error model enable flag.
*
* @param enable Enable/disable gyroscope stochastic error model.
*/
void ImuSimulator::setGyroEnableStochasticError(const bool enable) {
  imuModelEnableSettings_.gyro.enableStochasticError = enable;
}

/**
* @brief Setter for the gyroscope saturation model enable flag.
*
* @param enable Enable/disable gyroscope saturation model.
*/
void ImuSimulator::setGyroEnableSaturation(const bool enable) {
  imuModelEnableSettings_.gyro.enableSaturation = enable;
}

/**
* @brief Setter for the gyroscope quantization model enable flag.
*
* @param enable Enable/disable gyroscope quantization model.
*/
void ImuSimulator::setGyroEnableQuantization(const bool enable) {
  imuModelEnableSettings_.gyro.enableQuantization = enable;
}

/**
 * @brief Setter for the flag indicating if fixed random numbers are used.
 * 
 * @param enable Enable/disable fixed random numbers.
*/
void ImuSimulator::setUseFixedRandomNumbersFlag(const bool enable) {
  useFixedRandomNumbers_ = enable;
}

/**
 * @brief Initializes the IMU constant errors.
 * 
 * This function initializes the IMU constant errors by drawing random numbers from a uniform distribution.
 * The following IMU constant errors are initialized:
 * - constant turn-on bias,
 * - scale factor and misalignment.
*/
void ImuSimulator::initializeRandomConstantErrors() {
  // Draw const turn-on bias for each accelerometer axis
  accConstTurnOnBias_(0) =
      drawRandUniformDistNumFromInterval(accSimParams_.intervalTurnOnBias.x());
  accConstTurnOnBias_(1) =
      drawRandUniformDistNumFromInterval(accSimParams_.intervalTurnOnBias.y());
  accConstTurnOnBias_(2) =
      drawRandUniformDistNumFromInterval(accSimParams_.intervalTurnOnBias.z());

  // Draw const turn-on bias for each gyroscope axis
  gyroConstTurnOnBias_(0) =
      drawRandUniformDistNumFromInterval(gyroSimParams_.intervalTurnOnBias.x());
  gyroConstTurnOnBias_(1) =
      drawRandUniformDistNumFromInterval(gyroSimParams_.intervalTurnOnBias.y());
  gyroConstTurnOnBias_(2) =
      drawRandUniformDistNumFromInterval(gyroSimParams_.intervalTurnOnBias.z());

  // Draw const scale factor error for each accelerometer axis
  accConstScaleFactor_(0) =
      drawRandUniformDistNumFromInterval(accSimParams_.intervalScaleFactor.x());
  accConstScaleFactor_(1) =
      drawRandUniformDistNumFromInterval(accSimParams_.intervalScaleFactor.y());
  accConstScaleFactor_(2) =
      drawRandUniformDistNumFromInterval(accSimParams_.intervalScaleFactor.z());

  // Draw const scale factor for each gyroscope axis
  gyroConstScaleFactor_(0) = drawRandUniformDistNumFromInterval(
      gyroSimParams_.intervalScaleFactor.x());
  gyroConstScaleFactor_(1) = drawRandUniformDistNumFromInterval(
      gyroSimParams_.intervalScaleFactor.y());
  gyroConstScaleFactor_(2) = drawRandUniformDistNumFromInterval(
      gyroSimParams_.intervalScaleFactor.z());

  // Draw orthogonality error for each pair of accelerometer axes
  accConstNonOrthogonality_(0) = drawRandUniformDistNumFromInterval(
      accSimParams_.intervalNonOrthogonality[0]);  // xz-plane
  accConstNonOrthogonality_(1) = drawRandUniformDistNumFromInterval(
      accSimParams_.intervalNonOrthogonality[1]);  // xy-plane
  accConstNonOrthogonality_(2) = drawRandUniformDistNumFromInterval(
      accSimParams_.intervalNonOrthogonality[2]);  // yz-plane
  accConstNonOrthogonality_(3) = drawRandUniformDistNumFromInterval(
      accSimParams_.intervalNonOrthogonality[3]);  // yx-plane
  accConstNonOrthogonality_(4) = drawRandUniformDistNumFromInterval(
      accSimParams_.intervalNonOrthogonality[4]);  // zy-plane
  accConstNonOrthogonality_(5) = drawRandUniformDistNumFromInterval(
      accSimParams_.intervalNonOrthogonality[5]);  // zx-plane

  // Draw orthogonality error for each pair of gyroscope axes
  gyroConstNonOrthogonality_(0) = drawRandUniformDistNumFromInterval(
      gyroSimParams_.intervalNonOrthogonality[0]);  // xz-plane
  gyroConstNonOrthogonality_(1) = drawRandUniformDistNumFromInterval(
      gyroSimParams_.intervalNonOrthogonality[1]);  // xy-plane
  gyroConstNonOrthogonality_(2) = drawRandUniformDistNumFromInterval(
      gyroSimParams_.intervalNonOrthogonality[2]);  // yz-plane
  gyroConstNonOrthogonality_(3) = drawRandUniformDistNumFromInterval(
      gyroSimParams_.intervalNonOrthogonality[3]);  // yx-plane
  gyroConstNonOrthogonality_(4) = drawRandUniformDistNumFromInterval(
      gyroSimParams_.intervalNonOrthogonality[4]);  // zy-plane
  gyroConstNonOrthogonality_(5) = drawRandUniformDistNumFromInterval(
      gyroSimParams_.intervalNonOrthogonality[5]);  // zx-plane

  // Draw alignment error for each accelerometer axis
  accConstMisAlignment_(0) = drawRandUniformDistNumFromInterval(
      accSimParams_.intervalMisAlignment.x());
  accConstMisAlignment_(1) = drawRandUniformDistNumFromInterval(
      accSimParams_.intervalMisAlignment.y());
  accConstMisAlignment_(2) = drawRandUniformDistNumFromInterval(
      accSimParams_.intervalMisAlignment.z());

  // Draw alignment error for each gyroscope axis
  gyroConstMisAlignment_(0) = drawRandUniformDistNumFromInterval(
      gyroSimParams_.intervalMisAlignment.x());
  gyroConstMisAlignment_(1) = drawRandUniformDistNumFromInterval(
      gyroSimParams_.intervalMisAlignment.y());
  gyroConstMisAlignment_(2) = drawRandUniformDistNumFromInterval(
      gyroSimParams_.intervalMisAlignment.z());

  constErrorsInitialized_ = true;
}

/**
 * @brief Generates an erroneous accelerometer measurement vector.
 * 
 * This function generates an erroneous accelerometer measurement vector.
 * The measurement vector is generated by applying the following models:
 * - gravity model,
 * - scale factor,
 * - misalignment model,
 * - constant turn-on bias model,
 * - stochastic error model,
 * - saturation model,
 * - quantization model.
 * 
 * @param a_ib_b_true True acceleration of body frame with respect to inertial frame.
 * @param q_b_n_true True quaternion of body frame with respect to navigation frame.
*/
Eigen::Vector3d ImuSimulator::generateAccelerationMeasurement(
    const Eigen::Vector3d& a_ib_b_true, const Eigen::Quaterniond q_b_n_true) {
  // Check if constant errors have been initialized
  if (constErrorsInitialized_ == false) {
    std::cerr << "generateAccelerationMeasurement: Constant errors have not "
                 "been initialized!\n";
    return Eigen::Vector3d(0.0, 0.0, 0.0);
  }

  // Initialize measurement vector with true acceleration
  Eigen::Vector3d f_ib_b_meas = a_ib_b_true;

  // Update local gravity vector for last geodetic position
  environmentalParameters_.g_n = calcGravityVectorWelmec(geoPositionLlh_);

  // Apply gravity model, now acceleration is a specific force
  if (imuModelEnableSettings_.acc.enableLocalRefVec == true) {
    f_ib_b_meas = calcAccelerometerGravityModel(f_ib_b_meas, q_b_n_true);
  }

  // Apply constant turn-on bias model
  if (imuModelEnableSettings_.acc.enableTurnOnBias == true) {
    f_ib_b_meas = calcConstTurnOnBiasModel(f_ib_b_meas, acc);
  }

  // Apply scale factor error model
  if (imuModelEnableSettings_.acc.enableScaling == true) {
    f_ib_b_meas = calcScaleFactorModel(f_ib_b_meas, accConstScaleFactor_);
  }

  // Apply scale factor error and misalignment model
  if (imuModelEnableSettings_.acc.enableMisAlignment == true) {
    f_ib_b_meas = calcMisalignmentModel(f_ib_b_meas, accConstNonOrthogonality_,
                                        accConstMisAlignment_);
  }

  // Apply stochastic error model
  if (imuModelEnableSettings_.acc.enableStochasticError == true) {
    f_ib_b_meas = calcStochasticErrorModel(f_ib_b_meas, acc);
  }

  // Apply saturation model
  if (imuModelEnableSettings_.acc.enableSaturation == true) {
    f_ib_b_meas = calcSaturationModel(f_ib_b_meas, accSimParams_.measRange);
  }

  // Apply quantization model
  if (imuModelEnableSettings_.acc.enableQuantization == true) {
    f_ib_b_meas = calcQuantizationModel(f_ib_b_meas, accSimParams_.resolution);
  }

  return f_ib_b_meas;
}

/**
 * @brief Generates an erroneous gyroscope measurement vector.
 * 
 * This function generates an erroneous gyroscope measurement vector.
 * The measurement vector is generated by applying the following models:
 * - Earth angular velocity model,
 * - scale factor,
 * - misalignment model,
 * - constant turn-on bias model,
 * - stochastic error model,
 * - saturation model,
 * - quantization model.
 * 
 * @param w_nb_b_true True angular velocity of body frame with respect to navigation (NED) frame.
 * @param q_b_n_true True quaternion of body frame with respect to navigation frame.
*/
Eigen::Vector3d ImuSimulator::generateGyroscopeMeasurement(
    const Eigen::Vector3d& w_nb_b_true, const Eigen::Quaterniond q_b_n_true) {
  // Check if constant errors have been initialized
  if (constErrorsInitialized_ == false) {
    std::cerr << "generateGyroscopeMeasurement: Constant errors have not been "
                 "initialized!\n";
    return Eigen::Vector3d(0.0, 0.0, 0.0);
  }

  // Initialize measurement vector with true angular velocity
  Eigen::Vector3d w_ib_b_meas = w_nb_b_true;

  // Update local angular velocity of Earth for last geodetic position
  environmentalParameters_.w_ie_n = calcAngularVelocityOfEarth(geoPositionLlh_);

  // Update transport angular velocity for last geodetic position/velocity
  w_en_n_ = calcTransportAngularVelocity(geoPositionLlh_, v_eb_n_);

  // Apply Earth angular velocity model
  if (imuModelEnableSettings_.gyro.enableLocalRefVec == true) {
    w_ib_b_meas =
        calcGyroscopeEarthAngularVelocityModel(w_ib_b_meas, q_b_n_true);
  }

  // Apply constant turn-on bias model
  if (imuModelEnableSettings_.gyro.enableTurnOnBias == true) {
    w_ib_b_meas = calcConstTurnOnBiasModel(w_ib_b_meas, gyro);
  }

  // Apply scale factor error model
  if (imuModelEnableSettings_.gyro.enableScaling == true) {
    w_ib_b_meas = calcScaleFactorModel(w_ib_b_meas, gyroConstScaleFactor_);
  }

  // Apply scale factor error and misalignment model
  if (imuModelEnableSettings_.gyro.enableMisAlignment == true) {
    w_ib_b_meas = calcMisalignmentModel(w_ib_b_meas, gyroConstNonOrthogonality_,
                                        gyroConstMisAlignment_);
  }

  // Apply stochastic error model
  if (imuModelEnableSettings_.gyro.enableStochasticError == true) {
    w_ib_b_meas = calcStochasticErrorModel(w_ib_b_meas, gyro);
  }

  // Apply saturation model
  if (imuModelEnableSettings_.gyro.enableSaturation == true) {
    w_ib_b_meas = calcSaturationModel(w_ib_b_meas, gyroSimParams_.measRange);
  }

  // Apply quantization model
  if (imuModelEnableSettings_.gyro.enableQuantization == true) {
    w_ib_b_meas = calcQuantizationModel(w_ib_b_meas, gyroSimParams_.resolution);
  }

  return w_ib_b_meas;
}

/**
 * @brief Scaling error model for IMU measurements.
 * 
 * Scaling errors result from imperfect calibration of the sensor's scaling factor.
 * Since the sensor's onboard software already attempts to compensate for these errors, 
 * this function is meant to apply only to the uncompensated part.
 *
 * @param measurement Vector of IMU measurements (acceleration or angular velocity).
 * @param scaleFactor Scale factors in all three dimensions (%).
 * 
 * @return Measurement vector with scaling error.
 */
Eigen::Vector3d ImuSimulator::calcScaleFactorModel(
    const Eigen::Vector3d& measurement, const Eigen::Vector3d& scaleFactor) {
  // Apply scale factor error to measurement
  Eigen::Vector3d measurementScaled =
      measurement.array() * (1.0 + scaleFactor.array() / 100);

  return measurementScaled;
}

/**
 * @brief Misalignment and orthogonality error model for IMU measurements.
 * 
 * The IMU's measurements are subject to two further types of errors:
 * Non-orthogonality means that each of the axes can be tilted w.r.t. its perfect
 * orthogonal position independently from other axes (in addition to misalignment).
 * misalignment represents a rigid-body (orthogonal) rotational misplacement of
 * The actual sensor axes with their locations marked on the sensor.
 * This function applies all three of these errors to the measurement. Since the
 * The sensor's onboard software already attempts to compensate for these errors; this
 * function is meant to apply only to the uncompensated part.
 *
 * @param measurement Vector of IMU measurements (acceleration or angular velocity).
* @param nonOrthogonality Six angles between each pair of the axes (rad).
 * @param misAlignment Three Euler angles representing the rigid-body rotation (rad).
 * 
 * @return Measurement vector with misalignment and orthogonality error.
 */
Eigen::Vector3d ImuSimulator::calcMisalignmentModel(
    const Eigen::Vector3d& measurement,
    const Eigen::Matrix<double, 6, 1>& nonOrthogonality,
    const Eigen::Vector3d& misAlignment) {

  // Calculate misalignment matrix
  Eigen::Matrix3d misAlignmentMatrix{
      Eigen::AngleAxisd(misAlignment(0), Eigen::Vector3d::UnitX()) *
      Eigen::AngleAxisd(misAlignment(1), Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(misAlignment(2), Eigen::Vector3d::UnitZ())};

  // Calculate non-orthogonality matrix
  Eigen::Matrix3d nonOrthogonalityMatrix{
      {1.0, -nonOrthogonality(0), nonOrthogonality(1)},
      {nonOrthogonality(2), 1.0, -nonOrthogonality(3)},
      {-nonOrthogonality(4), nonOrthogonality(5), 1.0}};

  // Apply misalignment and orthogonality error to measurement
  Eigen::Vector3d misAlignedMeasurement =
      misAlignmentMatrix * nonOrthogonalityMatrix * measurement;

  return misAlignedMeasurement;
}

/**
* @brief Turn-on bias (constant bias) model for IMU measurements.
*
* An IMU shows a constant bias which changes every time the IMU is turned on.
* This function adds the constant turn-on bias to the measurement vector.
* The constant turn-on bias is drawn from a uniform distribution at class initialization.
*
* @param measurement Vector of IMU measurements (acceleration or angular velocity).
* @param measurementType Measurement type (acceleration or angular velocity).
*
* @return Measurement vector with added constant turn-on bias.
*/
Eigen::Vector3d ImuSimulator::calcConstTurnOnBiasModel(
    const Eigen::Vector3d& measurement,
    const MeasurementType& measurementType) {
  // Vector of measurement with constant turn-on bias
  Eigen::Vector3d measurementWithBias;

  if (constErrorsInitialized_ == false) {
    std::cerr << "calcConstTurnOnBiasModel: Constant errors not initialized!\n";
    return measurement;
  }

  // Add const turn-on bias to measurement
  if (measurementType == acc) {
    measurementWithBias = measurement + accConstTurnOnBias_;
  } else if (measurementType == gyro) {
    measurementWithBias = measurement + gyroConstTurnOnBias_;
  } else {
    // Invalid measurement type
    std::cerr << "calcConstTurnOnBiasModel: Invalid measurement type!\n";
    measurementWithBias = measurement;
  }

  return measurementWithBias;
}

/**
 * @brief Stochastic error model for IMU measurements.
 * 
 * This function adds the stochastic error to each axis of the measurement vector.
 * The detailed implementation is in function calcDiscreteTimeEquivalentModel().
 * 
 * @param measurement Vector of IMU measurements (acceleration or angular velocity).
 * @param measurementType Measurement type (acceleration or angular velocity).
 * @param normDistNumbersMatrix Matrix of normal distributed random numbers.
 * 
 * @return Measurement vector with added stochastic error.
*/
Eigen::Vector3d ImuSimulator::calcStochasticErrorModel(
    const Eigen::Vector3d& measurement,
    const MeasurementType& measurementType) {
  // Vector of measurement with stochastic error
  Eigen::Vector3d measurementWithStochasticError;

  // Vector of colored noise for each axis
  Eigen::Vector3d coloredNoise;

  // Update discrete-time equivalent state-space model for axes
  if (measurementType == acc) {
    // Get colored noise for each axis
    coloredNoise(0) = updateDiscreteTimeEquivalentStateSpaceModel(
        pAccStochasticErrors_, accSimParams_.N(axisX), accSimParams_.B(axisX),
        accSimParams_.K(axisX), accSimParams_.corrTime(axisX), axisX);
    coloredNoise(1) = updateDiscreteTimeEquivalentStateSpaceModel(
        pAccStochasticErrors_, accSimParams_.N(axisY), accSimParams_.B(axisY),
        accSimParams_.K(axisY), accSimParams_.corrTime(axisY), axisY);
    coloredNoise(2) = updateDiscreteTimeEquivalentStateSpaceModel(
        pAccStochasticErrors_, accSimParams_.N(axisZ), accSimParams_.B(axisZ),
        accSimParams_.K(axisZ), accSimParams_.corrTime(axisZ), axisZ);

    // Add stochastic error to measurement
    measurementWithStochasticError = measurement + coloredNoise;

  } else if (measurementType == gyro) {
    // Get colored noise for each axis
    coloredNoise(0) = updateDiscreteTimeEquivalentStateSpaceModel(
        pGyroStochasticErrors_, gyroSimParams_.N(axisX),
        gyroSimParams_.B(axisX), gyroSimParams_.K(axisX),
        gyroSimParams_.corrTime(axisX), axisX);
    coloredNoise(1) = updateDiscreteTimeEquivalentStateSpaceModel(
        pGyroStochasticErrors_, gyroSimParams_.N(axisY),
        gyroSimParams_.B(axisY), gyroSimParams_.K(axisY),
        gyroSimParams_.corrTime(axisY), axisY);
    coloredNoise(2) = updateDiscreteTimeEquivalentStateSpaceModel(
        pGyroStochasticErrors_, gyroSimParams_.N(axisZ),
        gyroSimParams_.B(axisZ), gyroSimParams_.K(axisZ),
        gyroSimParams_.corrTime(axisZ), axisZ);

    // Add stochastic error to measurement
    measurementWithStochasticError = measurement + coloredNoise;

  } else {
    // Invalid measurement type
    std::cerr << "calcStochasticErrorModel: Invalid measurement type!\n";
    measurementWithStochasticError = measurement;
  }

  return measurementWithStochasticError;
}

/**
 * @brief Discrete-time equivalent (NBK) model of a Gauss-Markov process to model stochastic errors.
 * 
 * This function calculates the discrete-time equivalent (NBK) model using a first-order Gauss-Markov process.
 * An IMU shows stochastic errors which sum up to colored noise, modeled as:
 *  - velocity/angle random walk (N),
 *  - bias instability (B),
 *  - acceleration/rate random walk (K).
 * The internal Gauss-Markov state space model is updated with these parameters if this function is called.
 * The model has internal states for time-correleated bias instability and acceleration/rate random walk.
 * The white noise velocity/angle random walk is added at the end to the internal states.
 * The output of the Gauss-Markov model is then the sum of all noise components which yields colored noise.
 * The function returns the colored noise for one axis as double (scalar).
 * @see [Inertial Measurement Unit Error Modeling Tutorial](https://ieeexplore.ieee.org/document/9955423)
 * 
 * @param pImuStochasticErrors Pointer to struct of IMU stochastic error states.
 * @param N Velocity/angle random walk.
 * @param B Bias instability.
 * @param K Accel./rate random walk.
 * @param corrTime Correlation time of Gauss-Markov error model.
 * @param normDistNumbers Vector of normal distributed random numbers.
 * @param axisId Axis identifier.
 * 
 * @return Sum of noise states (colored noise).
*/
double ImuSimulator::updateDiscreteTimeEquivalentStateSpaceModel(
    std::shared_ptr<ImuStochasticErrors> pImuStochasticErrors, const double N,
    const double B, const double K, const double corrTime,
    const AxisIdentifier axisId) {
  // PSD velocity/angle random walk
  double S_N = N * N;

  // PSD bias instability
  double S_B = (2 * B * B * std::log(2)) / (M_PI * 0.4365 * 0.4365 * corrTime);

  // PSD accel./rate random walk
  double S_K = K * K;

  // Gauss-Markov time correlation constant
  double corrConstant = 1 / corrTime;

  // Discrete-time state-space model matrices
  Eigen::Matrix2d Phi_k;
  Phi_k << std::exp(-corrConstant * dt_), 0, 0, 1;

  Eigen::MatrixXd H(1, 2);
  H << 1, 1;

  // Discrete-time bias instability covariance
  double Q_B_k =
      S_B / (2 * corrConstant) * (1 - std::exp(-2 * corrConstant * dt_));

  // Discrete-time rate random walk covariance
  double Q_K_k = S_K * dt_;

  // Discrete-time measurement noise covariance (velocity/angle random walk)
  double Q_N_k = S_N / dt_;

  // Define random number vector
  Eigen::Vector3d randomNumbers;

  if (useFixedRandomNumbers_ == true) {
    // Draw fixed random numbers for testing/debugging
    randomNumbers = Eigen::Vector3d(1.0, 2.0, 3.0);
  } else {
    // Draw normal distributed random samples from white noise processes
    randomNumbers =
        Eigen::Vector3d(drawRandNormalDistNum(), drawRandNormalDistNum(),
                        drawRandNormalDistNum());
  }

  // Draw measurement noise scalar (velocity/angle random walk)
  Eigen::VectorXd nu_N_k =
      Eigen::VectorXd::Constant(1, std::sqrt(Q_N_k) * randomNumbers(0));

  // Draw process noise vector (bias instability and accel./rate random walk)
  Eigen::Vector2d w_BK_k = Eigen::Vector2d(std::sqrt(Q_B_k) * randomNumbers(1),
                                           std::sqrt(Q_K_k) * randomNumbers(2));

  // Create state vector for one axis
  double z_B = pImuStochasticErrors->z_B(axisId);
  double z_K = pImuStochasticErrors->z_K(axisId);

  Eigen::Vector2d x_k = Eigen::Vector2d(z_B, z_K);

  // Calculate new state vector
  Eigen::Vector2d x_k_1 = Phi_k * x_k + w_BK_k;

  // Calculate new measurement vector
  Eigen::VectorXd z_k = H * x_k + nu_N_k;

  // Update internal Gauss-Markov state space model
  pImuStochasticErrors->z_N(axisId) = z_k(0);
  pImuStochasticErrors->z_B(axisId) = x_k_1(0);
  pImuStochasticErrors->z_K(axisId) = x_k_1(1);

  // Return colored noise as double (scalar)
  return z_k(0);
}

/** 
* @brief Gravity model for IMU acceleration measurements.
*
* An accelerometer measures the sum of the true acceleration of the body frame and the gravity vector (specific force).
* The true specific force is calculated by subtracting the gravity vector from the true acceleration.
*
* @param a_ib_b True acceleration of body frame with respect to inertial frame.
* @param q_b_n_true True quaternion of body frame with respect to navigation frame.
*
* @return True specific force of body frame with respect to inertial frame.
*/
Eigen::Vector3d ImuSimulator::calcAccelerometerGravityModel(
    const Eigen::Vector3d& a_ib_b, const Eigen::Quaterniond& q_b_n_true) {
  // Transform quaternion to rotation matrix
  Eigen::Matrix3d C_n_b = q_b_n_true.toRotationMatrix().transpose();

  // Calculate true specific force by subtracting gravity
  Eigen::Vector3d f_ib_b = a_ib_b - C_n_b * environmentalParameters_.g_n;

  return f_ib_b;
}

/**
 * @brief Model for ideal gyroscope angular velocity measurements.
 * 
 * A gyroscope measures the true angular velocity of the body with respect to the inertial frame.
 * Earth is rotating with an angular velocity, which needs to be added to the true angular velocity.
 * The rotation of the navigation frame with respect to Earth also needs to be considered (transport angular velocity).
 * 
 * @param w_nb_b True angular velocity of body frame with respect to NED frame.
 * @param q_b_n_true True quaternion of body frame with respect to navigation frame.
 * 
 * @return True angular velocity of body frame with respect to inertial frame.
*/
Eigen::Vector3d ImuSimulator::calcGyroscopeEarthAngularVelocityModel(
    const Eigen::Vector3d& w_nb_b, const Eigen::Quaterniond& q_b_n_true) {
  // Transform quaternion to rotation matrix
  Eigen::Matrix3d C_n_b = q_b_n_true.toRotationMatrix().transpose();

  // Calculate true angular velocity by adding Earth/transport angular velocity
  Eigen::Vector3d w_ib_b =
      w_nb_b + C_n_b * (environmentalParameters_.w_ie_n + w_en_n_);

  return w_ib_b;
}

/**
 * @brief Calculate the gravity vector in the navigation frame at a given latitude and height.
 *
 * This function calculates the local gravity vector in the navigation (NED) frame using 
 * the European WELMEC gravity model at a given latitude and height.
 * @see [WELMEC Guide](https://www.welmec.org/welmec/documents/guides/2/WELMEC_Guide_2_v2015.pdf)
 *
 * @param geoPositionLlh Geodetic position (latitude, longitude, height) of body frame (IMU)
 * 
 * @return Local gravity vector in the navigation frame (m/s²).
 */
Eigen::Vector3d ImuSimulator::calcGravityVectorWelmec(
    const Eigen::Vector3d& geoPositionLlh) {
  // Gravity reference values
  const double g_0 = 9.780318;
  const double g_1 = 5.3024e-3;
  const double g_2 = 5.8e-6;
  const double g_3 = 3.085e-6;

  // Local gravity vector according WELMEC model
  Eigen::Vector3d g_n;
  g_n << 0.0, 0.0,
      g_0 * (1 + g_1 * std::pow(std::sin(geoPositionLlh(0)), 2) -
             g_2 * std::pow(std::sin(2 * geoPositionLlh(0)), 2)) -
          g_3 * geoPositionLlh(2);

  return g_n;
}

/**
 * @brief Calculate the angular velocity of Earth in the navigation frame at a given latitude.
 *
 * This function calculates the angular velocity of Earth in the navigation (NED) frame at a given latitude.
 * @see [Aided Navigation - GPS with High Rate Sensors](https://dl.acm.org/doi/10.5555/1594745)
 * 
 * @param geoPositionLlh Geodetic position (latitude, longitude, height) of body frame (IMU)
 * 
 * @return Angular velocity vector of Earth in the navigation frame (rad/s).
 */
Eigen::Vector3d ImuSimulator::calcAngularVelocityOfEarth(
    const Eigen::Vector3d& geoPositionLlh) {
  const double omega_ie = 7.292115e-5;  // Angular velocity of Earth (rad/s)

  // Angular velocity of Earth in navigation frame
  Eigen::Vector3d w_ie_n;
  w_ie_n << omega_ie * std::cos(geoPositionLlh(0)), 0.0,
      -omega_ie * std::sin(geoPositionLlh(0));

  return w_ie_n;
}

/**
 * @brief Calculate the transport angular velocity in the navigation frame.
 * 
 * This function calculates the transport angular velocity in the navigation (NED) frame.
 * 
 * @param geoPositionLlh Geodetic position (latitude, longitude, height) of body frame (IMU)
 * @param v_eb_n Velocity of the body (b) frame with respect to the ECEF (e) frame (m/s).
 * 
 * @return Transport angular velocity vector in the navigation (NED) frame (rad/s).
*/
Eigen::Vector3d ImuSimulator::calcTransportAngularVelocity(
    const Eigen::Vector3d& geoPositionLlh, const Eigen::Vector3d& v_eb_n) {
  // Semi-major axis of Earth (m)
  const double a = 6378137.0;

  // Semi-minor axis of Earth (m)
  const double b = 6356752.31424518;

  // Flattening of Earth
  const double f = (a - b) / a;

  // Eccentricity of Earth
  const double e = std::sqrt(f * (2 - f));

  // Radius of curvature in the meridian plane (m)
  const double R_n = a * (1 - e * e) /
                     std::pow((1 - (e * e) * (std::sin(geoPositionLlh(0)) *
                                              std::sin(geoPositionLlh(0)))),
                              1.5);

  // Radius of curvature in the prime vertical (m)
  const double R_e = a / std::sqrt(1 - (e * e) * (std::sin(geoPositionLlh(0)) *
                                                  std::sin(geoPositionLlh(0))));

  // Average radius of curvature of Earth (m)
  const double R_0 = std::sqrt(R_n * R_e);

  // Transport angular velocity (rad/s)
  Eigen::Vector3d w_en_n;
  w_en_n << v_eb_n(1) / (R_0 + geoPositionLlh(2)),
      -v_eb_n(0) / (R_0 + geoPositionLlh(2)),
      -(v_eb_n(1) * std::tan(geoPositionLlh(0))) / (R_0 + geoPositionLlh(2));

  return w_en_n;
}

/**
 * @brief Saturation model for IMU measurements.
 * 
 * This function saturates the measurement vector to the absolute values of the measRange vector.
 * This replicates that an IMU can only measure values within a certain range.
 * 
 * @param measurement Vector of IMU measurements (acceleration or angular velocity).
 * @param measRange Vector of measurement ranges (acceleration or angular velocity).
 * 
 * @return Saturated measurement vector.
*/
Eigen::Vector3d ImuSimulator::calcSaturationModel(
    const Eigen::Vector3d& measurement, const Eigen::Vector3d& measRange) {
  // Saturated measurement vector
  Eigen::Vector3d saturatedMeasurement;

  // Saturate elements of measurement to absolute values of measRange
  saturatedMeasurement(0) = std::min(std::abs(measurement(0)), measRange(0)) *
                            (measurement(0) >= 0 ? 1 : -1);
  saturatedMeasurement(1) = std::min(std::abs(measurement(1)), measRange(1)) *
                            (measurement(1) >= 0 ? 1 : -1);
  saturatedMeasurement(2) = std::min(std::abs(measurement(2)), measRange(2)) *
                            (measurement(2) >= 0 ? 1 : -1);

  return saturatedMeasurement;
}

/**
 * @brief Quantization model for IMU measurements.
 * 
 * This function quantizes the measurement vector to the resolution of the resolution vector.
 * This replicates that an IMU can only measure values with a certain resolution (Analog/Digital conversion).
 * 
 * @param measurement Vector of IMU measurements (acceleration or angular velocity).
 * @param resolution Vector of measurement resolutions (acceleration or angular velocity).
 * 
 * @return Quantized measurement vector.
*/
Eigen::Vector3d ImuSimulator::calcQuantizationModel(
    const Eigen::Vector3d& measurement, const Eigen::Vector3d& resolution) {
  // Quantized measurement vector
  Eigen::Vector3d quantizedMeasurement;

  // Quantize elements of measurement to resolution
  quantizedMeasurement(0) =
      std::round(measurement(0) / resolution(0)) * resolution(0);
  quantizedMeasurement(1) =
      std::round(measurement(1) / resolution(1)) * resolution(1);
  quantizedMeasurement(2) =
      std::round(measurement(2) / resolution(2)) * resolution(2);

  return quantizedMeasurement;
}

/**
* @brief Reset the IMU simulator.
*/
void ImuSimulator::resetImuSimulator() {
  // Reset IMU stochastic errors
  pAccStochasticErrors_->z_N = Eigen::Vector3d::Zero();
  pAccStochasticErrors_->z_B = Eigen::Vector3d::Zero();
  pAccStochasticErrors_->z_K = Eigen::Vector3d::Zero();

  pGyroStochasticErrors_->z_N = Eigen::Vector3d::Zero();
  pGyroStochasticErrors_->z_B = Eigen::Vector3d::Zero();
  pGyroStochasticErrors_->z_K = Eigen::Vector3d::Zero();

  // Reset flag indicating if IMU constant errors are initialized
  constErrorsInitialized_ = false;

  // Re-Initialize IMU constant errors
  initializeRandomConstantErrors();
}

/**
 * @brief Draws a random number from a normal distribution.
 * 
 * This function draws a random number from a normal distribution with the given mean and standard deviation.
 * 
 * @return Single random number from the normal distribution.
*/
double ImuSimulator::drawRandNormalDistNum() {
  // Generate normal distributed random number
  return normalDistribution_(randomNumberGenerator_);
}

/**
 * @brief Draws a random number from a uniform distribution.
 * 
 * This function draws a random number from a uniform distribution in a given interval [-interval, +interval].
 * 
 * @param interval Interval of the uniform distribution.
 * @return Single random number from the uniform distribution.
*/
double ImuSimulator::drawRandUniformDistNumFromInterval(const double interval) {
  // Generate uniform distributed random number
  double randUniformNum = uniformDistribution_(randomNumberGenerator_);

  double a = -interval;
  double b = interval;

  return (b - a) * randUniformNum + a;
}

}  // namespace imu_simulator
