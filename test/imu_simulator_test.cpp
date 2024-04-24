/*@license BSD-3 https://opensource.org/licenses/BSD-3-Clause
Copyright (c) 2023, Institute of Automatic Control - RWTH Aachen University
Maximilian Nitsch (m.nitsch@irt.rwth-aachen.de)
All rights reserved.
*/

#include "imu_simulator.h"

#include "gtest/gtest.h"

/**
 * @brief Test fixture for the IMU simulator class
*/
class ImuSimulatorTest : public ::testing::Test {
 protected:
  void SetUp() override {
    const double gToMs2 = 9.80665;  // (g) -> (m/s^2)

    // Accelerometer simulation parameters, identified with for STIM300 IMU
    imu_simulator::ImuSimParams accSimParams;

    accSimParams.N = Eigen::Vector3d(0.00144813183998857, 0.00144218525538188,
                                     0.00145192879193856);
    accSimParams.B = Eigen::Vector3d(0.000259469978374161, 0.000284359672626612,
                                     0.000266572030700551);
    accSimParams.K = Eigen::Vector3d(9.72968195066955e-06, 1.50448806164571e-05,
                                     5.61621872457085e-06);
    accSimParams.corrTime =
        Eigen::Vector3d(44.2425330327468, 29.931356998205, 175.918924554171);

    accSimParams.intervalTurnOnBias =
        Eigen::Vector3d(5.0e-3, 5.0e-3, 5.0e-3) * gToMs2;  // (mg) -> (m/s^2)
    accSimParams.measRange =
        Eigen::Vector3d(5.0, 5.0, 5.0) * gToMs2;  // (g) -> (m/s^2)
    accSimParams.intervalMisAlignment =
        Eigen::Vector3d(0.2, 0.2, 0.2);  // (rad)
    accSimParams.intervalNonOrthogonality =
        Eigen::Matrix<double, 6, 1>(0.1, 0.1, 0.1, 0.1, 0.1, 0.1);      // (rad)
    accSimParams.intervalScaleFactor = Eigen::Vector3d(0.5, 0.5, 0.5);  // (%)
    accSimParams.resolution =
        Eigen::Vector3d(0.5960e-06, 0.5960e-06, 0.5960e-06) *
        gToMs2;  // (µg/LSB) -> (m/s^2)

    // Gyroscope simulation parameters, identified with for STIM300 IMU
    imu_simulator::ImuSimParams gyroSimParams;

    gyroSimParams.N = Eigen::Vector3d(
        0.000207479860047933, 0.000243411420514079, 0.000187943045943727);
    gyroSimParams.B = Eigen::Vector3d(8.601448819572e-07, 9.49331921834923e-07,
                                      9.20618084887883e-07);
    gyroSimParams.K = Eigen::Vector3d(1.85470342042531e-07, 1.1738725498127e-07,
                                      2.26095960190654e-07);
    gyroSimParams.corrTime = Eigen::Vector3d(1315.9775377824, 5263.99037881102,
                                             124.14459722317);  // (s)
    gyroSimParams.intervalTurnOnBias = Eigen::Vector3d(100.0, 100.0, 100.0) *
                                       M_PI / 180.0 * 1 /
                                       3600;  // (deg/h) -> (rad/s)
    gyroSimParams.measRange =
        Eigen::Vector3d(400.0, 400.0, 400.0) * M_PI / 180.0;  // (rad/s)
    gyroSimParams.intervalMisAlignment =
        Eigen::Vector3d(0.2, 0.2, 0.2);  // (rad)
    gyroSimParams.intervalNonOrthogonality =
        Eigen::Matrix<double, 6, 1>(0.1, 0.1, 0.1, 0.1, 0.1, 0.1);  // (rad)
    gyroSimParams.intervalScaleFactor = Eigen::Vector3d(0.5, 0.5, 0.5);  // (%)
    gyroSimParams.resolution =
        Eigen::Vector3d(0.832e-06, 0.832e-06, 0.832e-06);  // (rad/LSB)

    // IMU model enable settings
    imu_simulator::ImuModelEnableSettings imuModelEnableSettings;

    imuModelEnableSettings.acc.enableLocalRefVec = true;
    imuModelEnableSettings.acc.enableTurnOnBias = false;
    imuModelEnableSettings.acc.enableScaling = false;
    imuModelEnableSettings.acc.enableMisAlignment = false;
    imuModelEnableSettings.acc.enableStochasticError = false;
    imuModelEnableSettings.acc.enableSaturation = false;
    imuModelEnableSettings.acc.enableQuantization = false;

    imuModelEnableSettings.gyro.enableLocalRefVec = true;
    imuModelEnableSettings.gyro.enableTurnOnBias = false;
    imuModelEnableSettings.gyro.enableScaling = false;
    imuModelEnableSettings.gyro.enableMisAlignment = false;
    imuModelEnableSettings.gyro.enableStochasticError = false;
    imuModelEnableSettings.gyro.enableSaturation = false;
    imuModelEnableSettings.gyro.enableQuantization = false;

    // Geographic position of Neumayer III station
    double latitude = -70.667997328 * M_PI / 180.0;
    double longitude = -8.266998932 * M_PI / 180.0;
    double height = 40.0;

    Eigen::Vector3d geoPositionLlh =
        Eigen::Vector3d(latitude, longitude, height);

    // Neglect velocity of body frame (IMU) with respect to ECEF frame
    Eigen::Vector3d geoVelocity = Eigen::Vector3d(0.0, 0.0, 0.0);

    // IMU sample time
    double sampleTime = 0.01;

    // Random number generator seed
    unsigned int seed = 42;

    // Initialize the IMU simulator class
    imuSimulator = imu_simulator::ImuSimulator(
        accSimParams, gyroSimParams, imuModelEnableSettings, geoPositionLlh,
        geoVelocity, sampleTime, seed);
  }

  // Declare the class under test
  imu_simulator::ImuSimulator imuSimulator;
};

/**
 * @brief Test if the WELMEC model for the gravity vector outputs the same as MATLAB
*/
TEST_F(ImuSimulatorTest, WelmecModelGravityTest) {
  Eigen::Vector3d g_n_expected = Eigen::Vector3d(0.0, 0.0, 9.8263484510466);

  imu_simulator::EnvironmentalParameters environmentalParameters =
      imuSimulator.getEnvironmentalParameters();

  Eigen::Vector3d g_n = environmentalParameters.g_n;

  EXPECT_NEAR(g_n(0), g_n_expected(0), 1e-10);
  EXPECT_NEAR(g_n(1), g_n_expected(1), 1e-10);
  EXPECT_NEAR(g_n(2), g_n_expected(2), 1e-10);
}

/**
 * @brief Test if the gravity vector is correctly substracted from the acceleration measurement
*/
TEST_F(ImuSimulatorTest, AccGravityVectorTest) {
  Eigen::Vector3d f_ib_b = Eigen::Vector3d(0.0, 0.0, 0.0);
  Eigen::Quaterniond q_b_n = Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0);

  imu_simulator::EnvironmentalParameters environmentalParameters =
      imuSimulator.getEnvironmentalParameters();

  Eigen::Vector3d f_ib_b_expected =
      f_ib_b - q_b_n.toRotationMatrix() * environmentalParameters.g_n;

  Eigen::Vector3d f_ib_b_measured =
      imuSimulator.generateAccelerationMeasurement(f_ib_b, q_b_n);

  EXPECT_NEAR(f_ib_b_measured(0), f_ib_b_expected(0), 1e-10);
  EXPECT_NEAR(f_ib_b_measured(1), f_ib_b_expected(1), 1e-10);
  EXPECT_NEAR(f_ib_b_measured(2), f_ib_b_expected(2), 1e-10);
}

/**
 * @brief Test if the Earth angular rate is correctly added to the gyroscope measurement
*/
TEST_F(ImuSimulatorTest, GyroEarthAngularVelocityVectorTest) {
  Eigen::Vector3d w_ib_b = Eigen::Vector3d(0.0, 0.0, 0.0);
  Eigen::Quaterniond q_b_n = Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0);

  imu_simulator::EnvironmentalParameters environmentalParameters =
      imuSimulator.getEnvironmentalParameters();

  Eigen::Vector3d w_ib_b_expected =
      w_ib_b + q_b_n.toRotationMatrix() * environmentalParameters.w_ie_n;

  Eigen::Vector3d w_ib_b_measured =
      imuSimulator.generateGyroscopeMeasurement(w_ib_b, q_b_n);

  EXPECT_EQ(w_ib_b_measured(0), w_ib_b_expected(0));
  EXPECT_EQ(w_ib_b_measured(1), w_ib_b_expected(1));
  EXPECT_EQ(w_ib_b_measured(2), w_ib_b_expected(2));
}

/**
 * @brief Test if the accelerometer turn-on bias is always in the configured interval
*/
TEST_F(ImuSimulatorTest, AccTurnOnBiasInIntervalTest) {
  // Enable the accelerometer turn-on bias model
  imuSimulator.setAccEnableTurnOnBias(true);

  // Extract the configured interval for the accelerometer turn-on bias
  imu_simulator::ImuSimParams accSimParams =
      imuSimulator.getImuSimParams(imu_simulator::MeasurementType::acc);
  Eigen::Vector3d intervalTurnOnBiasAcc = accSimParams.intervalTurnOnBias;

  // Loop 10000 times to check if turn-on bias is always in configured interval
  for (int i = 0; i < 10000; i++) {
    Eigen::Vector3d turnOnBias =
        imuSimulator.getConstTurnOnBias(imu_simulator::MeasurementType::acc);

    EXPECT_LE(turnOnBias.x(), intervalTurnOnBiasAcc.x());
    EXPECT_LE(turnOnBias.y(), intervalTurnOnBiasAcc.y());
    EXPECT_LE(turnOnBias.z(), intervalTurnOnBiasAcc.z());

    // Reset the IMU simulator to draw new random turn-on biases
    imuSimulator.resetImuSimulator();
  }
}

/**
 * @brief Test if gyroscope turn-on bias is always in configured interval
*/
TEST_F(ImuSimulatorTest, GyroTurnOnBiasInIntervalTest) {
  // Enable the gyroscope turn-on bias model
  imuSimulator.setGyroEnableTurnOnBias(true);

  // Extract the configured interval for the gyroscope turn-on bias
  imu_simulator::ImuSimParams gyroSimParams =
      imuSimulator.getImuSimParams(imu_simulator::MeasurementType::gyro);
  Eigen::Vector3d intervalTurnOnBiasGyro = gyroSimParams.intervalTurnOnBias;

  // Loop 10000 times to check if turn-on bias is always in configured interval
  for (int i = 0; i < 10000; i++) {
    Eigen::Vector3d turnOnBias =
        imuSimulator.getConstTurnOnBias(imu_simulator::MeasurementType::gyro);

    EXPECT_LE(turnOnBias.x(), intervalTurnOnBiasGyro.x());
    EXPECT_LE(turnOnBias.y(), intervalTurnOnBiasGyro.y());
    EXPECT_LE(turnOnBias.z(), intervalTurnOnBiasGyro.z());

    // Reset the IMU simulator to draw new random turn-on biases
    imuSimulator.resetImuSimulator();
  }
}

/**
 * @brief Test if the accelerometer constant turn-on bias remains constant
*/
TEST_F(ImuSimulatorTest, AccTurnOnBiasRemainsConstantTest) {
  // Enable the accelerometer turn-on bias model
  imuSimulator.setAccEnableTurnOnBias(true);

  Eigen::Vector3d turnOnBiasExpected =
      imuSimulator.getConstTurnOnBias(imu_simulator::MeasurementType::acc);

  // Loop 5 times to check if turn-on bias is always in configured interval
  for (int i = 0; i < 5; i++) {
    imuSimulator.generateAccelerationMeasurement(
        Eigen::Vector3d(0.0, 0.0, 0.0), Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0));

    Eigen::Vector3d turnOnBias =
        imuSimulator.getConstTurnOnBias(imu_simulator::MeasurementType::acc);

    // Turn-on bias should remain constant
    EXPECT_EQ(turnOnBias.x(), turnOnBiasExpected.x());
    EXPECT_EQ(turnOnBias.y(), turnOnBiasExpected.y());
    EXPECT_EQ(turnOnBias.z(), turnOnBiasExpected.z());
  }

  // Reset the IMU simulator to draw new random turn-on biases
  imuSimulator.resetImuSimulator();

  Eigen::Vector3d turnOnBias =
      imuSimulator.getConstTurnOnBias(imu_simulator::MeasurementType::acc);

  // Now the turn-on bias should be different
  EXPECT_NE(turnOnBias.x(), turnOnBiasExpected.x());
  EXPECT_NE(turnOnBias.y(), turnOnBiasExpected.y());
  EXPECT_NE(turnOnBias.z(), turnOnBiasExpected.z());
}

/**
 * @brief Test if the gyroscope constant turn-on bias remains constant
*/
TEST_F(ImuSimulatorTest, GyroTurnOnBiasRemainsConstantTest) {
  // Enable the gyroscope turn-on bias model
  imuSimulator.setGyroEnableTurnOnBias(true);

  Eigen::Vector3d turnOnBiasExpected =
      imuSimulator.getConstTurnOnBias(imu_simulator::MeasurementType::gyro);

  // Loop 5 times to check if turn-on bias is always in configured interval
  for (int i = 0; i < 5; i++) {
    imuSimulator.generateGyroscopeMeasurement(
        Eigen::Vector3d(0.0, 0.0, 0.0), Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0));

    Eigen::Vector3d turnOnBias =
        imuSimulator.getConstTurnOnBias(imu_simulator::MeasurementType::gyro);

    // Turn-on bias should remain constant
    EXPECT_EQ(turnOnBias.x(), turnOnBiasExpected.x());
    EXPECT_EQ(turnOnBias.y(), turnOnBiasExpected.y());
    EXPECT_EQ(turnOnBias.z(), turnOnBiasExpected.z());
  }

  // Reset the IMU simulator to draw new random turn-on biases
  imuSimulator.resetImuSimulator();

  Eigen::Vector3d turnOnBias =
      imuSimulator.getConstTurnOnBias(imu_simulator::MeasurementType::gyro);

  // Now the turn-on bias should be different
  EXPECT_NE(turnOnBias.x(), turnOnBiasExpected.x());
  EXPECT_NE(turnOnBias.y(), turnOnBiasExpected.y());
  EXPECT_NE(turnOnBias.z(), turnOnBiasExpected.z());
}

/**
 * @brief Test if the accelerometer measurements are correctly scaled and misaligned
*/
TEST_F(ImuSimulatorTest, AccScalingModelTest) {
  // Set the true test acceleration and orientation
  Eigen::Vector3d f_ib_b = Eigen::Vector3d(1.0, 2.0, 3.0);
  Eigen::Quaterniond q_b_n = Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0);

  // Complete the baseline measurement (without misalignment and co)
  Eigen::Vector3d f_ib_b_baseline =
      imuSimulator.generateAccelerationMeasurement(f_ib_b, q_b_n);

  // Enable scaling model
  imuSimulator.setAccEnableScaling(true);

  // Reset to draw new random scaling values
  imuSimulator.resetImuSimulator();

  // Get the scale factor error, misalignment and orthogonality error
  Eigen::Vector3d scaleFactor =
      imuSimulator.getConstScaleFactor(imu_simulator::MeasurementType::acc);

  // Calculate the expected measurement for the present scaling values
  Eigen::Vector3d f_ib_b_expected =
      f_ib_b_baseline.array() * (1.0 + scaleFactor.array() / 100);

  // Get the accelerometer measurement and compare it with the expected value
  Eigen::Vector3d f_ib_b_meas{
      imuSimulator.generateAccelerationMeasurement(f_ib_b, q_b_n)};

  EXPECT_NEAR(f_ib_b_expected(0), f_ib_b_meas(0), 1e-10);
  EXPECT_NEAR(f_ib_b_expected(1), f_ib_b_meas(1), 1e-10);
  EXPECT_NEAR(f_ib_b_expected(2), f_ib_b_meas(2), 1e-10);
}

/**
 * @brief Test if the accelerometer measurements are correctly misaligned
*/
TEST_F(ImuSimulatorTest, AccMisAlignmentModelTest) {
  // Set the true test acceleration and orientation
  Eigen::Vector3d f_ib_b = Eigen::Vector3d(1.0, 2.0, 3.0);
  Eigen::Quaterniond q_b_n = Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0);

  // Complete the baseline measurement (without misalignment and co)
  Eigen::Vector3d f_ib_b_baseline =
      imuSimulator.generateAccelerationMeasurement(f_ib_b, q_b_n);

  // Enable misalignment model
  imuSimulator.setAccEnableMisAlignment(true);

  // Reset to draw new random misalignment values
  imuSimulator.resetImuSimulator();

  // Get the misalignment and orthogonality error
  Eigen::Vector3d misAlignment =
      imuSimulator.getConstMisAlignment(imu_simulator::MeasurementType::acc);

  Eigen::Matrix<double, 6, 1> nonOrthogonality =
      imuSimulator.getConstNonOrthogonality(
          imu_simulator::MeasurementType::acc);

  // Calculate the expected measurement for the present misalignment values
  Eigen::Matrix3d misAlignmentMatrix{
      Eigen::AngleAxisd(misAlignment(0), Eigen::Vector3d::UnitX()) *
      Eigen::AngleAxisd(misAlignment(1), Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(misAlignment(2), Eigen::Vector3d::UnitZ())};

  Eigen::Matrix3d nonOrthogonalityMatrix{
      {1.0, -nonOrthogonality(0), nonOrthogonality(1)},
      {nonOrthogonality(2), 1.0, -nonOrthogonality(3)},
      {-nonOrthogonality(4), nonOrthogonality(5), 1.0}};

  // Calculate the expected measurement for the present misalignment values
  Eigen::Vector3d f_ib_b_expected =
      misAlignmentMatrix * nonOrthogonalityMatrix * f_ib_b_baseline;

  // Get the accelerometer measurement and compare it with the expected value
  Eigen::Vector3d f_ib_b_meas{
      imuSimulator.generateAccelerationMeasurement(f_ib_b, q_b_n)};

  EXPECT_NEAR(f_ib_b_expected(0), f_ib_b_meas(0), 1e-10);
  EXPECT_NEAR(f_ib_b_expected(1), f_ib_b_meas(1), 1e-10);
  EXPECT_NEAR(f_ib_b_expected(2), f_ib_b_meas(2), 1e-10);
}

/**
 * @brief Test if the gyroscope measurements are correctly scaled
*/
TEST_F(ImuSimulatorTest, GyroScalingModelTest) {
  // Set the true test angular rate and orientation
  Eigen::Vector3d w_ib_b = Eigen::Vector3d(1.0, 2.0, 3.0);
  Eigen::Quaterniond q_b_n = Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0);

  // Complete the baseline measurement (without scaling)
  Eigen::Vector3d w_ib_b_baseline =
      imuSimulator.generateGyroscopeMeasurement(w_ib_b, q_b_n);

  // Enable scaling model
  imuSimulator.setGyroEnableScaling(true);

  // Reset to draw new random scaling values
  imuSimulator.resetImuSimulator();

  // Get the scale factor error
  Eigen::Vector3d scaleFactor =
      imuSimulator.getConstScaleFactor(imu_simulator::MeasurementType::gyro);

  // Calculate the expected measurement for the present misalignment values
  Eigen::Vector3d w_ib_b_expected =
      w_ib_b_baseline.array() * (1.0 + scaleFactor.array() / 100);

  // Get the gyro measurement and compare it with the expected value
  Eigen::Vector3d w_ib_b_meas =
      imuSimulator.generateGyroscopeMeasurement(w_ib_b, q_b_n);

  EXPECT_NEAR(w_ib_b_expected(0), w_ib_b_meas(0), 1e-10);
  EXPECT_NEAR(w_ib_b_expected(1), w_ib_b_meas(1), 1e-10);
  EXPECT_NEAR(w_ib_b_expected(2), w_ib_b_meas(2), 1e-10);
}

/**
 * @brief Test if the gyroscope measurements are correctly scaled
*/
TEST_F(ImuSimulatorTest, GyroMisAlignmentModelTest) {
  // Set the true test angular rate and orientation
  Eigen::Vector3d w_ib_b{Eigen::Vector3d(1.0, 2.0, 3.0)};
  Eigen::Quaterniond q_b_n{Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0)};

  // Complete the baseline measurement (without misalignment and co)
  Eigen::Vector3d w_ib_b_baseline{
      imuSimulator.generateGyroscopeMeasurement(w_ib_b, q_b_n)};

  // Enable misalignment model
  imuSimulator.setGyroEnableMisAlignment(true);

  // Reset to draw new random misalignment values
  imuSimulator.resetImuSimulator();

  // Get the misalignment and orthogonality error
  Eigen::Vector3d misAlignment =
      imuSimulator.getConstMisAlignment(imu_simulator::MeasurementType::gyro);

  Eigen::Matrix<double, 6, 1> nonOrthogonality =
      imuSimulator.getConstNonOrthogonality(
          imu_simulator::MeasurementType::gyro);

  // Calculate the expected measurement for the present misalignment values
  Eigen::Matrix3d misAlignmentMatrix{
      Eigen::AngleAxisd(misAlignment(0), Eigen::Vector3d::UnitX()) *
      Eigen::AngleAxisd(misAlignment(1), Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxisd(misAlignment(2), Eigen::Vector3d::UnitZ())};

  Eigen::Matrix3d nonOrthogonalityMatrix{
      {1.0, -nonOrthogonality(0), nonOrthogonality(1)},
      {nonOrthogonality(2), 1.0, -nonOrthogonality(3)},
      {-nonOrthogonality(4), nonOrthogonality(5), 1.0}};

  // Calculate the expected measurement for the present misalignment values
  Eigen::Vector3d w_ib_b_expected =
      misAlignmentMatrix * nonOrthogonalityMatrix * w_ib_b_baseline;

  // Get the gyro measurement and compare it with the expected value
  Eigen::Vector3d w_ib_b_meas{
      imuSimulator.generateGyroscopeMeasurement(w_ib_b, q_b_n)};

  EXPECT_NEAR(w_ib_b_expected(0), w_ib_b_meas(0), 1e-10);
  EXPECT_NEAR(w_ib_b_expected(1), w_ib_b_meas(1), 1e-10);
  EXPECT_NEAR(w_ib_b_expected(2), w_ib_b_meas(2), 1e-10);
}

/**
 * @brief Test if the accelerometer measurement is saturated to the configured range
*/
TEST_F(ImuSimulatorTest, AccSaturationModelTest) {
  // Enable the accelerometer turn-on bias model
  imuSimulator.setAccEnableSaturation(true);

  // Extract the configured interval for the accelerometer turn-on bias
  imu_simulator::ImuSimParams accSimParams =
      imuSimulator.getImuSimParams(imu_simulator::MeasurementType::acc);
  Eigen::Vector3d measRange = accSimParams.measRange;

  Eigen::Vector3d f_ib_b_meas = imuSimulator.generateAccelerationMeasurement(
      Eigen::Vector3d(2.0 * measRange(0), 2.0 * measRange(1),
                      2.0 * measRange(2)),
      Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0));

  // Accelerometer measurement should be saturated
  EXPECT_LE(f_ib_b_meas(0), measRange(0));
  EXPECT_LE(f_ib_b_meas(1), measRange(1));
  EXPECT_LE(f_ib_b_meas(2), measRange(2));

  // Disable the accelerometer turn-on bias model
  imuSimulator.setAccEnableSaturation(false);

  f_ib_b_meas = imuSimulator.generateAccelerationMeasurement(
      Eigen::Vector3d(2.0 * measRange(0), 2.0 * measRange(1),
                      2.0 * measRange(2)),
      Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0));

  // Accelerometer measurement should not be saturated
  EXPECT_GE(f_ib_b_meas(0), measRange(0));
  EXPECT_GE(f_ib_b_meas(1), measRange(1));
  EXPECT_GE(f_ib_b_meas(2), measRange(2));
}

/**
 * @brief Test if the gyroscope measurement is saturated to the configured range
*/
TEST_F(ImuSimulatorTest, GyroSaturationModelTest) {
  // Enable the gyroscope turn-on bias model
  imuSimulator.setGyroEnableSaturation(true);

  // Extract the configured interval for the gyroscope turn-on bias
  imu_simulator::ImuSimParams gyroSimParams =
      imuSimulator.getImuSimParams(imu_simulator::MeasurementType::gyro);
  Eigen::Vector3d measRange = gyroSimParams.measRange;

  Eigen::Vector3d w_ib_b_meas = imuSimulator.generateGyroscopeMeasurement(
      Eigen::Vector3d(2.0 * measRange(0), 2.0 * measRange(1),
                      2.0 * measRange(2)),
      Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0));

  // Gyroscope measurement should be saturated
  EXPECT_LE(w_ib_b_meas(0), measRange(0));
  EXPECT_LE(w_ib_b_meas(1), measRange(1));
  EXPECT_LE(w_ib_b_meas(2), measRange(2));

  // Disable the gyroscope turn-on bias model
  imuSimulator.setGyroEnableSaturation(false);

  w_ib_b_meas = imuSimulator.generateGyroscopeMeasurement(
      Eigen::Vector3d(2.0 * measRange(0), 2.0 * measRange(1),
                      2.0 * measRange(2)),
      Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0));

  // Gyroscope measurement should not be saturated
  EXPECT_GE(w_ib_b_meas(0), measRange(0));
  EXPECT_GE(w_ib_b_meas(1), measRange(1));
  EXPECT_GE(w_ib_b_meas(2), measRange(2));
}

/**
 * @brief Test if the accelerometer measurement is quantized to the configured resolution
*/
TEST_F(ImuSimulatorTest, AccQuantizationModelTest) {
  Eigen::Vector3d f_ib_b = Eigen::Vector3d(1.0, 2.0, 3.0);
  Eigen::Quaterniond q_b_n = Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0);

  // Disable the gravity vector model
  imuSimulator.setAccEnableGravity(false);

  // Enable the accelerometer quantization model
  imuSimulator.setAccEnableQuantization(true);

  // Extract the IMU simulation parameters
  imu_simulator::ImuSimParams accSimParams =
      imuSimulator.getImuSimParams(imu_simulator::MeasurementType::acc);
  imu_simulator::ImuSimParams gyroSimParams =
      imuSimulator.getImuSimParams(imu_simulator::MeasurementType::gyro);

  // Set the resolution to a high value
  accSimParams.resolution = Eigen::Vector3d(3, 0.5, 1.25);

  // Set new accelerometer simulation parameters
  imuSimulator.setImuSimParameters(accSimParams, gyroSimParams);

  // Expected quantized measurement
  Eigen::Vector3d f_ib_b_expected = Eigen::Vector3d(0.0, 2.0, 2.5);

  // Generate an accelerometer measurement
  Eigen::Vector3d f_ib_b_meas =
      imuSimulator.generateAccelerationMeasurement(f_ib_b, q_b_n);

  // Accelerometer measurement should be quantized
  EXPECT_EQ(f_ib_b_meas(0), f_ib_b_expected(0));
  EXPECT_EQ(f_ib_b_meas(1), f_ib_b_expected(1));
  EXPECT_EQ(f_ib_b_meas(2), f_ib_b_expected(2));
}

/**
 * @brief Test if the gyroscope measurement is quantized to the configured resolution
*/
TEST_F(ImuSimulatorTest, GyroQuantizationModelTest) {
  Eigen::Vector3d w_ib_b = Eigen::Vector3d(1.0, 2.0, 3.0);
  Eigen::Quaterniond q_b_n = Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0);

  // Disable the Earth angular velocity model
  imuSimulator.setGyroEnableEarthAngularVelocity(false);

  // Enable the gyroscope quantization model
  imuSimulator.setGyroEnableQuantization(true);

  // Extract the IMU simulation parameters
  imu_simulator::ImuSimParams accSimParams =
      imuSimulator.getImuSimParams(imu_simulator::MeasurementType::acc);
  imu_simulator::ImuSimParams gyroSimParams =
      imuSimulator.getImuSimParams(imu_simulator::MeasurementType::gyro);

  // Set the resolution to a high value
  gyroSimParams.resolution = Eigen::Vector3d(3, 0.5, 1.25);

  // Set new gyroscope simulation parameters
  imuSimulator.setImuSimParameters(accSimParams, gyroSimParams);

  // Expected quantized measurement
  Eigen::Vector3d w_ib_b_expected = Eigen::Vector3d(0.0, 2.0, 2.5);

  // Generate a gyroscope measurement
  Eigen::Vector3d w_ib_b_meas =
      imuSimulator.generateGyroscopeMeasurement(w_ib_b, q_b_n);

  // Gyroscope measurement should be quantized
  EXPECT_EQ(w_ib_b_meas(0), w_ib_b_expected(0));
  EXPECT_EQ(w_ib_b_meas(1), w_ib_b_expected(1));
  EXPECT_EQ(w_ib_b_meas(2), w_ib_b_expected(2));
}

/**
 * @brief Test if the accelerometer stochastic errors are zero when the noise parameters are zero
*/
TEST_F(ImuSimulatorTest, AccStochasticErrorModelZeroNoiseTest) {
  // Set accelerometer noise parameters to zero to disable the noise
  imu_simulator::ImuSimParams accSimParams;

  accSimParams.N = Eigen::Vector3d(0.0, 0.0, 0.0);
  accSimParams.B = Eigen::Vector3d(0.0, 0.0, 0.0);
  accSimParams.K = Eigen::Vector3d(0.0, 0.0, 0.0);
  accSimParams.corrTime = Eigen::Vector3d(100.0, 100.0, 100.0);

  // Set  gyro noise parameters to zero to disable the noise
  imu_simulator::ImuSimParams gyroSimParams;

  gyroSimParams.N = Eigen::Vector3d(0.0, 0.0, 0.0);
  gyroSimParams.B = Eigen::Vector3d(0.0, 0.0, 0.0);
  gyroSimParams.K = Eigen::Vector3d(0.0, 0.0, 0.0);
  gyroSimParams.corrTime = Eigen::Vector3d(100.0, 100.0, 100.0);

  // Overwrite the IMU simulation parameters with zero noise parameters
  imuSimulator.setImuSimParameters(accSimParams, gyroSimParams);

  // Disable the gravity vector model
  imuSimulator.setAccEnableGravity(false);

  // Enable the accelerometer stochastic error model
  imuSimulator.setAccEnableStochasticError(true);

  Eigen::Vector3d f_ib_b_meas;
  imu_simulator::ImuStochasticErrors imuStochasticErrors;

  // Generate an accelerometer measurement
  for (int i = 0; i < 1000; i++) {
    f_ib_b_meas = imuSimulator.generateAccelerationMeasurement(
        Eigen::Vector3d(0.0, 0.0, 0.0), Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0));

    imuStochasticErrors = imuSimulator.getImuStochasticErrors(
        imu_simulator::MeasurementType::acc);
  }

  // Accelerometer measurement should be zero
  EXPECT_EQ(f_ib_b_meas(0), 0.0);
  EXPECT_EQ(f_ib_b_meas(1), 0.0);
  EXPECT_EQ(f_ib_b_meas(2), 0.0);

  // Accelerometer stochastic errors should be zero
  EXPECT_EQ(imuStochasticErrors.z_N(0), 0.0);
  EXPECT_EQ(imuStochasticErrors.z_N(1), 0.0);
  EXPECT_EQ(imuStochasticErrors.z_N(2), 0.0);

  EXPECT_EQ(imuStochasticErrors.z_B(0), 0.0);
  EXPECT_EQ(imuStochasticErrors.z_B(1), 0.0);
  EXPECT_EQ(imuStochasticErrors.z_B(2), 0.0);

  EXPECT_EQ(imuStochasticErrors.z_K(0), 0.0);
  EXPECT_EQ(imuStochasticErrors.z_K(1), 0.0);
  EXPECT_EQ(imuStochasticErrors.z_K(2), 0.0);
}

/**
 * @brief Test if the gyroscope stochastic errors are zero when the noise parameters are zero
*/
TEST_F(ImuSimulatorTest, GyroStochasticErrorModelZeroNoiseTest) {
  // Set accelerometer noise parameters to zero to disable the noise
  imu_simulator::ImuSimParams accSimParams;

  accSimParams.N = Eigen::Vector3d(0.0, 0.0, 0.0);
  accSimParams.B = Eigen::Vector3d(0.0, 0.0, 0.0);
  accSimParams.K = Eigen::Vector3d(0.0, 0.0, 0.0);
  accSimParams.corrTime = Eigen::Vector3d(100.0, 100.0, 100.0);

  // Set  gyro noise parameters to zero to disable the noise
  imu_simulator::ImuSimParams gyroSimParams;

  gyroSimParams.N = Eigen::Vector3d(0.0, 0.0, 0.0);
  gyroSimParams.B = Eigen::Vector3d(0.0, 0.0, 0.0);
  gyroSimParams.K = Eigen::Vector3d(0.0, 0.0, 0.0);
  gyroSimParams.corrTime = Eigen::Vector3d(100.0, 100.0, 100.0);

  // Overwrite the IMU simulation parameters with zero noise parameters
  imuSimulator.setImuSimParameters(accSimParams, gyroSimParams);

  // Disable the Earth angular velocity model
  imuSimulator.setGyroEnableEarthAngularVelocity(false);

  // Enable the gyroscope stochastic error model
  imuSimulator.setGyroEnableStochasticError(true);

  Eigen::Vector3d w_ib_b_meas;
  imu_simulator::ImuStochasticErrors imuStochasticErrors;

  // Generate a gyroscope measurement
  for (int i = 0; i < 1000; i++) {
    w_ib_b_meas = imuSimulator.generateGyroscopeMeasurement(
        Eigen::Vector3d(0.0, 0.0, 0.0), Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0));

    imuStochasticErrors = imuSimulator.getImuStochasticErrors(
        imu_simulator::MeasurementType::gyro);
  }

  // Gyroscope measurement should be zero
  EXPECT_EQ(w_ib_b_meas(0), 0.0);
  EXPECT_EQ(w_ib_b_meas(1), 0.0);
  EXPECT_EQ(w_ib_b_meas(2), 0.0);

  // Gyroscopes stochastic errors should be zero
  EXPECT_EQ(imuStochasticErrors.z_N(0), 0.0);
  EXPECT_EQ(imuStochasticErrors.z_N(1), 0.0);
  EXPECT_EQ(imuStochasticErrors.z_N(2), 0.0);

  EXPECT_EQ(imuStochasticErrors.z_B(0), 0.0);
  EXPECT_EQ(imuStochasticErrors.z_B(1), 0.0);
  EXPECT_EQ(imuStochasticErrors.z_B(2), 0.0);

  EXPECT_EQ(imuStochasticErrors.z_K(0), 0.0);
  EXPECT_EQ(imuStochasticErrors.z_K(1), 0.0);
  EXPECT_EQ(imuStochasticErrors.z_K(2), 0.0);
}

/**
 * @brief Test if the accelerometer stochastic error match the manual calculations for fixed random numbers
*/
TEST_F(ImuSimulatorTest, AccStochasticErrorModelTest) {
  // Set accelerometer noise parameters to zero to disable the noise
  imu_simulator::ImuSimParams accSimParams;

  accSimParams.N = Eigen::Vector3d(0.001, 0.001, 0.001);
  accSimParams.B = Eigen::Vector3d(0.0001, 0.0001, 0.0001);
  accSimParams.K = Eigen::Vector3d(1e-5, 1e-5, 1e-5);
  accSimParams.corrTime = Eigen::Vector3d(100.0, 100.0, 100.0);

  // Set  gyro noise parameters to zero to disable the noise
  imu_simulator::ImuSimParams gyroSimParams;

  gyroSimParams.N = Eigen::Vector3d(0.0, 0.0, 0.0);
  gyroSimParams.B = Eigen::Vector3d(0.0, 0.0, 0.0);
  gyroSimParams.K = Eigen::Vector3d(0.0, 0.0, 0.0);
  gyroSimParams.corrTime = Eigen::Vector3d(100.0, 100.0, 100.0);

  // Overwrite the IMU simulation parameters with zero noise parameters
  imuSimulator.setImuSimParameters(accSimParams, gyroSimParams);

  // Disable the gravity vector model
  imuSimulator.setAccEnableGravity(false);

  // Enable the accelerometer stochastic error model
  imuSimulator.setAccEnableStochasticError(true);

  // Set the random numbers to fixed values for testing
  imuSimulator.setUseFixedRandomNumbersFlag(true);

  Eigen::Vector3d f_ib_b_meas;
  f_ib_b_meas = imuSimulator.generateAccelerationMeasurement(
      Eigen::Vector3d(0.0, 0.0, 0.0), Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0));

  imu_simulator::ImuStochasticErrors imuStochasticErrors;
  imuStochasticErrors =
      imuSimulator.getImuStochasticErrors(imu_simulator::MeasurementType::acc);

  Eigen::Vector3d z_N_expected = Eigen::Vector3d(0.01, 0.01, 0.01);
  Eigen::Vector3d z_B_expected = Eigen::Vector3d(
      3.04352466221447e-06, 3.04352466221447e-06, 3.04352466221447e-06);
  Eigen::Vector3d z_K_expected = Eigen::Vector3d(3e-6, 3e-6, 3e-6);

  // Check if the stochastic errors are as expected for fixed random numbers
  EXPECT_NEAR(imuStochasticErrors.z_N(0), z_N_expected(0), 1e-10);
  EXPECT_NEAR(imuStochasticErrors.z_N(1), z_N_expected(1), 1e-10);
  EXPECT_NEAR(imuStochasticErrors.z_N(2), z_N_expected(2), 1e-10);

  EXPECT_NEAR(imuStochasticErrors.z_B(0), z_B_expected(0), 1e-10);
  EXPECT_NEAR(imuStochasticErrors.z_B(1), z_B_expected(1), 1e-10);
  EXPECT_NEAR(imuStochasticErrors.z_B(2), z_B_expected(2), 1e-10);

  EXPECT_NEAR(imuStochasticErrors.z_K(0), z_K_expected(0), 1e-10);
  EXPECT_NEAR(imuStochasticErrors.z_K(1), z_K_expected(1), 1e-10);
  EXPECT_NEAR(imuStochasticErrors.z_K(2), z_K_expected(2), 1e-10);

  // Generate second measurement
  f_ib_b_meas = imuSimulator.generateAccelerationMeasurement(
      Eigen::Vector3d(0.0, 0.0, 0.0), Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0));

  imuStochasticErrors =
      imuSimulator.getImuStochasticErrors(imu_simulator::MeasurementType::acc);

  // Check that stochastic errors changed after second measurement
  EXPECT_FALSE(imuStochasticErrors.z_N.isApprox(z_N_expected, 1e-10));
  EXPECT_FALSE(imuStochasticErrors.z_B.isApprox(z_B_expected, 1e-10));
  EXPECT_FALSE(imuStochasticErrors.z_K.isApprox(z_K_expected, 1e-10));

  // Reset the IMU simulator to draw new random numbers
  imuSimulator.resetImuSimulator();

  f_ib_b_meas = imuSimulator.generateAccelerationMeasurement(
      Eigen::Vector3d(0.0, 0.0, 0.0), Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0));

  imuStochasticErrors =
      imuSimulator.getImuStochasticErrors(imu_simulator::MeasurementType::acc);

  // Check if stochastic errors are as expected for fixed random numbers
  EXPECT_NEAR(imuStochasticErrors.z_N(0), z_N_expected(0), 1e-10);
  EXPECT_NEAR(imuStochasticErrors.z_N(1), z_N_expected(1), 1e-10);
  EXPECT_NEAR(imuStochasticErrors.z_N(2), z_N_expected(2), 1e-10);

  EXPECT_NEAR(imuStochasticErrors.z_B(0), z_B_expected(0), 1e-10);
  EXPECT_NEAR(imuStochasticErrors.z_B(1), z_B_expected(1), 1e-10);
  EXPECT_NEAR(imuStochasticErrors.z_B(2), z_B_expected(2), 1e-10);

  EXPECT_NEAR(imuStochasticErrors.z_K(0), z_K_expected(0), 1e-10);
  EXPECT_NEAR(imuStochasticErrors.z_K(1), z_K_expected(1), 1e-10);
  EXPECT_NEAR(imuStochasticErrors.z_K(2), z_K_expected(2), 1e-10);
}

/**
 * @brief Test if the gyroscope stochastic error match the manual calculations for fixed random numbers
 * 
*/
TEST_F(ImuSimulatorTest, GyroStochasticErrorModelTest) {
  // Set accelerometer noise parameters to zero to disable the noise
  imu_simulator::ImuSimParams accSimParams;

  accSimParams.N = Eigen::Vector3d(0.0, 0.0, 0.0);
  accSimParams.B = Eigen::Vector3d(0.0, 0.0, 0.0);
  accSimParams.K = Eigen::Vector3d(0.0, 0.0, 0.0);
  accSimParams.corrTime = Eigen::Vector3d(100.0, 100.0, 100.0);

  // Set  gyro noise parameters to zero to disable the noise
  imu_simulator::ImuSimParams gyroSimParams;

  gyroSimParams.N = Eigen::Vector3d(0.001, 0.001, 0.001);
  gyroSimParams.B = Eigen::Vector3d(0.0001, 0.0001, 0.0001);
  gyroSimParams.K = Eigen::Vector3d(1e-5, 1e-5, 1e-5);
  gyroSimParams.corrTime = Eigen::Vector3d(100.0, 100.0, 100.0);

  // Overwrite the IMU simulation parameters with zero noise parameters
  imuSimulator.setImuSimParameters(accSimParams, gyroSimParams);

  // Disable the Earth angular velocity model
  imuSimulator.setGyroEnableEarthAngularVelocity(false);

  // Enable the gyroscope stochastic error model
  imuSimulator.setGyroEnableStochasticError(true);

  // Set the random numbers to fixed values for testing
  imuSimulator.setUseFixedRandomNumbersFlag(true);

  Eigen::Vector3d w_ib_b_meas;
  w_ib_b_meas = imuSimulator.generateGyroscopeMeasurement(
      Eigen::Vector3d(0.0, 0.0, 0.0), Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0));

  imu_simulator::ImuStochasticErrors imuStochasticErrors;
  imuStochasticErrors =
      imuSimulator.getImuStochasticErrors(imu_simulator::MeasurementType::gyro);

  Eigen::Vector3d z_N_expected = Eigen::Vector3d(0.01, 0.01, 0.01);
  Eigen::Vector3d z_B_expected = Eigen::Vector3d(
      3.04352466221447e-06, 3.04352466221447e-06, 3.04352466221447e-06);
  Eigen::Vector3d z_K_expected = Eigen::Vector3d(3e-6, 3e-6, 3e-6);

  // Check if the stochastic errors are as expected for fixed random numbers
  EXPECT_NEAR(imuStochasticErrors.z_N(0), z_N_expected(0), 1e-10);
  EXPECT_NEAR(imuStochasticErrors.z_N(1), z_N_expected(1), 1e-10);
  EXPECT_NEAR(imuStochasticErrors.z_N(2), z_N_expected(2), 1e-10);

  EXPECT_NEAR(imuStochasticErrors.z_B(0), z_B_expected(0), 1e-10);
  EXPECT_NEAR(imuStochasticErrors.z_B(1), z_B_expected(1), 1e-10);
  EXPECT_NEAR(imuStochasticErrors.z_B(2), z_B_expected(2), 1e-10);

  EXPECT_NEAR(imuStochasticErrors.z_K(0), z_K_expected(0), 1e-10);
  EXPECT_NEAR(imuStochasticErrors.z_K(1), z_K_expected(1), 1e-10);
  EXPECT_NEAR(imuStochasticErrors.z_K(2), z_K_expected(2), 1e-10);

  // Generate second measurement
  w_ib_b_meas = imuSimulator.generateGyroscopeMeasurement(
      Eigen::Vector3d(0.0, 0.0, 0.0), Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0));

  imuStochasticErrors =
      imuSimulator.getImuStochasticErrors(imu_simulator::MeasurementType::gyro);

  // Check that stochastic errors changed after second measurement
  EXPECT_FALSE(imuStochasticErrors.z_N.isApprox(z_N_expected, 1e-10));
  EXPECT_FALSE(imuStochasticErrors.z_B.isApprox(z_B_expected, 1e-10));
  EXPECT_FALSE(imuStochasticErrors.z_K.isApprox(z_K_expected, 1e-10));

  // Reset the IMU simulator to draw new random numbers
  imuSimulator.resetImuSimulator();

  w_ib_b_meas = imuSimulator.generateGyroscopeMeasurement(
      Eigen::Vector3d(0.0, 0.0, 0.0), Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0));

  imuStochasticErrors =
      imuSimulator.getImuStochasticErrors(imu_simulator::MeasurementType::gyro);

  // Check if stochastic errors are as expected for fixed random numbers
  EXPECT_NEAR(imuStochasticErrors.z_N(0), z_N_expected(0), 1e-10);
  EXPECT_NEAR(imuStochasticErrors.z_N(1), z_N_expected(1), 1e-10);
  EXPECT_NEAR(imuStochasticErrors.z_N(2), z_N_expected(2), 1e-10);

  EXPECT_NEAR(imuStochasticErrors.z_B(0), z_B_expected(0), 1e-10);
  EXPECT_NEAR(imuStochasticErrors.z_B(1), z_B_expected(1), 1e-10);
  EXPECT_NEAR(imuStochasticErrors.z_B(2), z_B_expected(2), 1e-10);

  EXPECT_NEAR(imuStochasticErrors.z_K(0), z_K_expected(0), 1e-10);
  EXPECT_NEAR(imuStochasticErrors.z_K(1), z_K_expected(1), 1e-10);
  EXPECT_NEAR(imuStochasticErrors.z_K(2), z_K_expected(2), 1e-10);
}
