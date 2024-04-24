/*@license BSD-3 https://opensource.org/licenses/BSD-3-Clause
Copyright (c) 2024, Institute of Automatic Control - RWTH Aachen University
Maximilian Nitsch (m.nitsch@irt.rwth-aachen.de)
All rights reserved.
*/

#include <fstream>
#include <iomanip>
#include <iostream>

// NOLINTNEXTLINE(build/c++11)
#include <chrono>
// NOLINTNEXTLINE(build/c++11)
#include <thread>

#include "imu_simulator.h"

// Write data to a CSV file
void writeToCsvFile(const std::vector<std::string>& data) {
  std::ofstream outputFile;

  outputFile.open("imu_static_data.csv", std::ios_base::app);

  // Check if the file is open
  if (!outputFile.is_open()) {
    std::cerr << "Error opening the file." << std::endl;
    return;
  }

  // add a comma at the end of each element except the last one
  for (size_t i = 0; i < data.size(); i++) {
    outputFile << data[i];

    if (i < data.size() - 1) {
      outputFile << ",";
    }
  }

  // add a new line after each row
  outputFile << "\n";

  // Close the file
  outputFile.close();
}

// Convert an Eigen vector to a string
std::string eigenVectorToString(const Eigen::VectorXd& eigenVector) {
  std::ostringstream oss;

  for (int i = 0; i < eigenVector.size(); i++) {
    oss << eigenVector[i];

    if (i < eigenVector.size() - 1) {
      oss << ",";
    }
  }

  return oss.str();
}

// Convert StateVector to a vector of strings
std::vector<std::string> imuMeasurementsToString(
    const Eigen::VectorXd& eigenVector1, const Eigen::VectorXd& eigenVector2) {
  std::vector<std::string> eigenVectorSTring;

  // Convert each Eigen vector/quaternion to a string
  eigenVectorSTring.push_back(eigenVectorToString(eigenVector1));
  eigenVectorSTring.push_back(eigenVectorToString(eigenVector2));

  return eigenVectorSTring;
}

// Main function
int main(int argc, char** argv) {
  (void)argc;
  (void)argv;

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
  accSimParams.intervalMisAlignment = Eigen::Vector3d(0.0, 0.0, 0.0);
  accSimParams.intervalScaleFactor = Eigen::Vector3d(0.0, 0.0, 0.0);
  accSimParams.resolution =
      Eigen::Vector3d(0.5960e-06, 0.5960e-06, 0.5960e-06) *
      gToMs2;  // (µg/LSB) -> (m/s^2)

  // Gyroscope simulation parameters, identified with for STIM300 IMU
  imu_simulator::ImuSimParams gyroSimParams;

  gyroSimParams.N = Eigen::Vector3d(0.000207479860047933, 0.000243411420514079,
                                    0.000187943045943727);
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
  gyroSimParams.intervalMisAlignment = Eigen::Vector3d(0.0, 0.0, 0.0);
  gyroSimParams.intervalScaleFactor = Eigen::Vector3d(0.0, 0.0, 0.0);
  gyroSimParams.resolution =
      Eigen::Vector3d(0.832e-06, 0.832e-06, 0.832e-06);  // (rad/LSB)

  // IMU model enable settings
  imu_simulator::ImuModelEnableSettings imuModelEnableSettings;

  imuModelEnableSettings.acc.enableLocalRefVec = true;
  imuModelEnableSettings.acc.enableTurnOnBias =
      false;  // disable turn-on bias for validation
  imuModelEnableSettings.acc.enableScaling = false;
  imuModelEnableSettings.acc.enableMisAlignment = false;
  imuModelEnableSettings.acc.enableStochasticError = true;
  imuModelEnableSettings.acc.enableSaturation = false;
  imuModelEnableSettings.acc.enableQuantization = false;

  imuModelEnableSettings.gyro.enableLocalRefVec = true;
  imuModelEnableSettings.gyro.enableTurnOnBias =
      false;  // disable turn-on bias for validation
  imuModelEnableSettings.gyro.enableScaling = false;
  imuModelEnableSettings.gyro.enableMisAlignment = false;
  imuModelEnableSettings.gyro.enableStochasticError = true;
  imuModelEnableSettings.gyro.enableSaturation = false;
  imuModelEnableSettings.gyro.enableQuantization = false;

  // Geographic position of Neumayer III station
  double latitude = -70.667997328 * M_PI / 180.0;
  double longitude = -8.266998932 * M_PI / 180.0;
  double height = 40.0;

  Eigen::Vector3d geoPositionLlh = Eigen::Vector3d(latitude, longitude, height);

  // Neglect the velocity of the body frame (IMU) with respect to the ECEF frame
  Eigen::Vector3d geoVelocity = Eigen::Vector3d(0.0, 0.0, 0.0);

  // IMU sample time
  double sampleTime = 1.0 / 125.0;

  // Random number generator seed
  unsigned int seed = 40;

  // Initialize the IMU simulator class
  imu_simulator::ImuSimulator imuSimulator = imu_simulator::ImuSimulator(
      accSimParams, gyroSimParams, imuModelEnableSettings, geoPositionLlh,
      geoVelocity, sampleTime, seed);

  imuSimulator.setUseFixedRandomNumbersFlag(false);

  imuSimulator.resetImuSimulator();

  std::stringstream ss = imuSimulator.printImuSimulatorParameters();
  std::cout << ss.str();

  // Wait for 5 seconds
  std::this_thread::sleep_for(std::chrono::seconds(5));

  // Accelerometer measurement
  Eigen::Vector3d a_ib_b_true = Eigen::Vector3d(0.0, 0.0, 0.0);
  Eigen::Vector3d w_ib_b_true = Eigen::Vector3d(0.0, 0.0, 0.0);

  Eigen::Quaterniond q_b_n_true = Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0);

  // Duration of the simulation
  double simTime = 8 * 60 * 60;  // 8h

  std::cout << "Starting static IMU simulation..." << std::endl;

  // Wait for 2 second
  std::this_thread::sleep_for(std::chrono::seconds(2));

  for (int i = 0; i < simTime / sampleTime; i++) {
    Eigen::Vector3d a_ib_b_meas =
        imuSimulator.generateAccelerationMeasurement(a_ib_b_true, q_b_n_true);

    Eigen::Vector3d w_ib_b_meas =
        imuSimulator.generateGyroscopeMeasurement(w_ib_b_true, q_b_n_true);

    std::cout << std::setw(10) << std::setprecision(10)
              << a_ib_b_meas.transpose() << std::endl;

    // Create string vector for the IMU measurements
    std::vector<std::string> stateVectorString =
        imuMeasurementsToString(a_ib_b_meas, w_ib_b_meas);

    // Write the IMU measurements to a csv file
    writeToCsvFile(stateVectorString);
  }

  std::cout << "Finished static IMU simulation!" << std::endl;

  imuSimulator.printImuSimulatorParameters();

  return 0;
}
