/*@license BSD-3 https://opensource.org/licenses/BSD-3-Clause
Copyright (c) 2024, Institute of Automatic Control - RWTH Aachen University
Maximilian Nitsch (m.nitsch@irt.rwth-aachen.de)
All rights reserved.
*/

// NOLINTNEXTLINE(build/c++11)
#include <chrono>
#include <cstdio>
#include <random>

#include "imu_simulator_node.h"

namespace imu_simulator {

/**
 * @brief IMU simulator node constructor with default config file path
 * 
 * @param pImuSimulator Pointer to the IMU simulator object
*/
ImuSimulatorNode::ImuSimulatorNode(std::shared_ptr<ImuSimulator> pImuSimulator)
    : Node("imu_simulator_node"),
      pImuSimulator_(pImuSimulator),
      groundTruthOdomMsg_(nullptr),
      groundTruthAccelMsg_(nullptr),
      lastOdomTimestamp_(0, 0),
      lastAccelTimestamp_(0, 0),
      sampleTime_(0.0),
      first_odometry_received_(false),
      odometry_timeout_(false),
      first_acceleration_received_(false),
      acceleration_timeout_(false),
      calc_acc_from_odom_vel_(false),
      firstLinearVelocityReceived_(false),
      timeLastOdom_(now()) {
  RCLCPP_INFO(get_logger(), "Configuring IMU simulator node...");

  // Declare adnd retrieve parameters and load them to IMU simulator
  declareAndRetrieveGeneralSettings();
  declareAndRetrieveImuParameters();
  declareAndRetrieveEnableSettings();
  declareAndRetrieveEnvironmentalSettings();

  RCLCPP_INFO(get_logger(), "Parameters from YAML config loaded successfully.");

  // Reset IMU simulator to ensure that constant errors are initialized
  pImuSimulator_->resetImuSimulator();

  // Print IMU simulator parameters
  std::stringstream ss = pImuSimulator_->printImuSimulatorParameters();
  RCLCPP_INFO(get_logger(), "%s", ss.str().c_str());

  // Get odometry topic name from launch file or use default
  std::string groundTruthOdometryTopicName;

  // Get the value of the topic_name parameter
  this->declare_parameter("topic_name_odom", rclcpp::PARAMETER_STRING);

  this->get_parameter_or("topic_name_odom", groundTruthOdometryTopicName,
                         std::string("/nanoauv/odometry"));

  RCLCPP_INFO(get_logger(), "Subscribing to ground truth odometry topic: %s",
              groundTruthOdometryTopicName.c_str());

  // Initialize the odometry subscriber
  pOdometrySubscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
      groundTruthOdometryTopicName, 10,
      std::bind(&ImuSimulatorNode::odometryCallback, this,
                std::placeholders::_1));

  // Create subscriber for ground truth acceleration if enabled
  if (!calc_acc_from_odom_vel_) {
    // Get acceleration topic name from launch file or use default
    std::string groundTruthAccelTopicName;
    this->get_parameter_or("topic_name_accel", groundTruthAccelTopicName,
                           std::string("/nanoauv/accel"));

    this->declare_parameter("topic_name_accel", rclcpp::PARAMETER_STRING);

    RCLCPP_INFO(get_logger(),
                "Subscribing to ground truth acceleration topic: %s",
                groundTruthAccelTopicName.c_str());

    // Initialize the acceleration subscriber
    pAccelerationSubscriber_ =
        this->create_subscription<geometry_msgs::msg::AccelStamped>(
            groundTruthAccelTopicName, 10,
            std::bind(&ImuSimulatorNode::accelerationCallback, this,
                      std::placeholders::_1));
  } else {
    // Initialize the ground truth acceleration message with zeros to circumvent null pointer
    groundTruthAccelMsg_ = std::make_shared<geometry_msgs::msg::AccelStamped>();

    groundTruthAccelMsg_->header.stamp = now();
    groundTruthAccelMsg_->header.frame_id = "base_link";

    groundTruthAccelMsg_->accel.linear.x = 0.0;
    groundTruthAccelMsg_->accel.linear.y = 0.0;
    groundTruthAccelMsg_->accel.linear.z = 0.0;

    groundTruthAccelMsg_->accel.angular.x = 0.0;
    groundTruthAccelMsg_->accel.angular.y = 0.0;
    groundTruthAccelMsg_->accel.angular.z = 0.0;

    first_acceleration_received_ = true;

    lastAccelTimestamp_ = now();

    RCLCPP_INFO(
        get_logger(),
        "Ground truth acceleration topic is not used. Acceleration is "
        "calculated by numerical differentiation of odometry velocity.");
  }

  // Retrieve the namespace from the node
  std::string nsStr = get_namespace();

  // Initialize the IMU data publisher
  pImuPublisher_ =
      this->create_publisher<sensor_msgs::msg::Imu>(nsStr + "/data", 10);

  // Initialize the IMU visualization publisher
  pImuAccelStampedPublisher_ =
      this->create_publisher<geometry_msgs::msg::AccelStamped>(
          nsStr + "/data_visualization", 10);

  // Initialize the acceleration publisher
  pTrueAccelerationPublisher_ =
      this->create_publisher<geometry_msgs::msg::Vector3>(
          nsStr + "/true_linear_acceleration", 10);

  // Initialize the angular rate publisher
  pTrueAngularRatePublisher_ =
      this->create_publisher<geometry_msgs::msg::Vector3>(
          nsStr + "/true_linear_angular_velocity", 10);

  // Initialize the diagnostic status publisher
  pDiagnosticPublisher_ =
      this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
          nsStr + "/diagnostic", 10);

  // Initialize the tf2 broadcaster
  pStaticTf2Broadcaster_ =
      std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

  // Publish static tf2 transformations
  this->publishTf2Transforms();

  // Set IMU sample time with data from the parameter server
  sampleTime_ =
      get_parameter("imu_simulator.general_settings.sample_time").as_double();

  // Convert sample time to milliseconds and cast to int
  int sampleTimeInt = static_cast<int>(sampleTime_ * 1e3);

  RCLCPP_INFO(get_logger(), "IMU simulator node executing with %dms.",
              sampleTimeInt);

  // Create a timer to call the IMU simulator loop callback function
  pTimer_ = this->create_wall_timer(
      std::chrono::milliseconds(sampleTimeInt),
      std::bind(&ImuSimulatorNode::imuSimulatorLoopCallback, this));

  // Create timer for odometry subscriber
  pOdometryTimeOutTimer_ = this->create_wall_timer(
      std::chrono::seconds(5),
      std::bind(&ImuSimulatorNode::odometryTimeOutCallback, this));

  if (!calc_acc_from_odom_vel_) {
    // Create timer for acceleration subscriber
    pAccelerationTimeOutTimer_ = this->create_wall_timer(
        std::chrono::seconds(5),
        std::bind(&ImuSimulatorNode::accelerationTimeOutCallback, this));
  }

  RCLCPP_INFO(get_logger(), "IMU simulator node initialized.");

  if (calc_acc_from_odom_vel_) {
    RCLCPP_INFO(get_logger(),
                "IMU simulator node waiting for first odometry message");
  } else {
    RCLCPP_INFO(
        get_logger(),
        "IMU simulator node waiting for first odometry and acceleration "
        "message...");
  }
}

/**
 * @brief Declare and retrieve general settings from the parameter server
*/
void ImuSimulatorNode::declareAndRetrieveGeneralSettings() {
  // General settings
  double sampleTime;
  int seed;
  bool useConstantSeed;

  // Declare general settings
  this->declare_parameter("imu_simulator.general_settings.sample_time",
                          rclcpp::PARAMETER_DOUBLE);
  this->declare_parameter("imu_simulator.general_settings.seed",
                          rclcpp::PARAMETER_INTEGER);
  this->declare_parameter("imu_simulator.general_settings.use_constant_seed",
                          rclcpp::PARAMETER_BOOL);
  this->declare_parameter(
      "imu_simulator.general_settings."
      "calc_accel_from_odometry_velocity",
      rclcpp::PARAMETER_BOOL);

  // Retrieve general settings
  sampleTime = this->get_parameter("imu_simulator.general_settings.sample_time")
                   .as_double();
  seed = this->get_parameter("imu_simulator.general_settings.seed").as_int();
  useConstantSeed =
      this->get_parameter("imu_simulator.general_settings.use_constant_seed")
          .as_bool();
  calc_acc_from_odom_vel_ = this->get_parameter(
                                    "imu_simulator.general_settings."
                                    "calc_accel_from_odometry_velocity")
                                .as_bool();

  // Set IMU simulator sample time and seed
  pImuSimulator_->setImuSampleTime(sampleTime);

  // Set seed depending on the useConstantSeed flag
  if (!useConstantSeed) {
    // Draw a random seed from the random device
    std::random_device rd;
    seed = rd();
    pImuSimulator_->setSimulatorSeed(seed);
    RCLCPP_INFO(get_logger(), "Using random seed: %d", seed);
  } else {
    // Set the random number generator seed
    pImuSimulator_->setSimulatorSeed(seed);
    RCLCPP_INFO(get_logger(), "Using seed from config file: %d", seed);
  }
}

/**
 * @brief Declare and retrieve IMU model parameters from the parameter server
*/
void ImuSimulatorNode::declareAndRetrieveImuParameters() {
  // Accelerometer and gyroscope model parameters
  ImuSimParams accSimParams;
  ImuSimParams gyroSimParams;

  // Declare model parameters accelerometer
  this->declare_parameter(
      "imu_simulator.model_parameter_settings.accelerometer.N",
      rclcpp::PARAMETER_DOUBLE_ARRAY);
  this->declare_parameter(
      "imu_simulator.model_parameter_settings.accelerometer.B",
      rclcpp::PARAMETER_DOUBLE_ARRAY);
  this->declare_parameter(
      "imu_simulator.model_parameter_settings.accelerometer.K",
      rclcpp::PARAMETER_DOUBLE_ARRAY);
  this->declare_parameter(
      "imu_simulator.model_parameter_settings.accelerometer.correlation_time",
      rclcpp::PARAMETER_DOUBLE_ARRAY);
  this->declare_parameter(
      "imu_simulator.model_parameter_settings.accelerometer.interval_turn_on_"
      "bias",
      rclcpp::PARAMETER_DOUBLE_ARRAY);
  this->declare_parameter(
      "imu_simulator.model_parameter_settings.accelerometer.measurement_range",
      rclcpp::PARAMETER_DOUBLE_ARRAY);
  this->declare_parameter(
      "imu_simulator.model_parameter_settings.accelerometer.interval_"
      "misalignment",
      rclcpp::PARAMETER_DOUBLE_ARRAY);
  this->declare_parameter(
      "imu_simulator.model_parameter_settings.accelerometer.interval_non_"
      "orthogonality",
      rclcpp::PARAMETER_DOUBLE_ARRAY);
  this->declare_parameter(
      "imu_simulator.model_parameter_settings.accelerometer.interval_scale_"
      "factor",
      rclcpp::PARAMETER_DOUBLE_ARRAY);
  this->declare_parameter(
      "imu_simulator.model_parameter_settings.accelerometer.resolution",
      rclcpp::PARAMETER_DOUBLE_ARRAY);

  // Declare model parameters gyroscope
  this->declare_parameter("imu_simulator.model_parameter_settings.gyroscope.N",
                          rclcpp::PARAMETER_DOUBLE_ARRAY);

  this->declare_parameter("imu_simulator.model_parameter_settings.gyroscope.B",
                          rclcpp::PARAMETER_DOUBLE_ARRAY);

  this->declare_parameter("imu_simulator.model_parameter_settings.gyroscope.K",
                          rclcpp::PARAMETER_DOUBLE_ARRAY);
  this->declare_parameter(
      "imu_simulator.model_parameter_settings.gyroscope.correlation_time",
      rclcpp::PARAMETER_DOUBLE_ARRAY);
  this->declare_parameter(
      "imu_simulator.model_parameter_settings.gyroscope.interval_turn_on_bias",
      rclcpp::PARAMETER_DOUBLE_ARRAY);
  this->declare_parameter(
      "imu_simulator.model_parameter_settings.gyroscope.measurement_range",
      rclcpp::PARAMETER_DOUBLE_ARRAY);
  this->declare_parameter(
      "imu_simulator.model_parameter_settings.gyroscope.interval_misalignment",
      rclcpp::PARAMETER_DOUBLE_ARRAY);
  this->declare_parameter(
      "imu_simulator.model_parameter_settings.gyroscope.interval_non_"
      "orthogonality",
      rclcpp::PARAMETER_DOUBLE_ARRAY);
  this->declare_parameter(
      "imu_simulator.model_parameter_settings.gyroscope.interval_scale_factor",
      rclcpp::PARAMETER_DOUBLE_ARRAY);
  this->declare_parameter(
      "imu_simulator.model_parameter_settings.gyroscope.resolution",
      rclcpp::PARAMETER_DOUBLE_ARRAY);

  // Retrieve model parameters accelerometer
  std::vector<double> accN =
      this->get_parameter(
              "imu_simulator.model_parameter_settings.accelerometer.N")
          .as_double_array();
  std::vector<double> accB =
      this->get_parameter(
              "imu_simulator.model_parameter_settings.accelerometer.B")
          .as_double_array();
  std::vector<double> accK =
      this->get_parameter(
              "imu_simulator.model_parameter_settings.accelerometer.K")
          .as_double_array();
  std::vector<double> accCorrTime =
      this->get_parameter(
              "imu_simulator.model_parameter_settings.accelerometer."
              "correlation_time")
          .as_double_array();
  std::vector<double> accIntervalTurnOnBias =
      this->get_parameter(
              "imu_simulator.model_parameter_settings.accelerometer.interval_"
              "turn_on_bias")
          .as_double_array();
  std::vector<double> accMeasRange =
      this->get_parameter(
              "imu_simulator.model_parameter_settings.accelerometer."
              "measurement_range")
          .as_double_array();
  std::vector<double> accIntervalMisAlignment =
      this->get_parameter(
              "imu_simulator.model_parameter_settings.accelerometer.interval_"
              "misalignment")
          .as_double_array();
  std::vector<double> accIntervalNonOrthogonality =
      this->get_parameter(
              "imu_simulator.model_parameter_settings.accelerometer.interval_"
              "non_orthogonality")
          .as_double_array();
  std::vector<double> accIntervalScaleFactor =
      this->get_parameter(
              "imu_simulator.model_parameter_settings.accelerometer.interval_"
              "scale_factor")
          .as_double_array();
  std::vector<double> accResolution =
      this->get_parameter(
              "imu_simulator.model_parameter_settings.accelerometer.resolution")
          .as_double_array();

  // Retrieve model parameters gyroscope
  std::vector<double> gyroN =
      this->get_parameter("imu_simulator.model_parameter_settings.gyroscope.N")
          .as_double_array();
  std::vector<double> gyroB =
      this->get_parameter("imu_simulator.model_parameter_settings.gyroscope.B")
          .as_double_array();
  std::vector<double> gyroK =
      this->get_parameter("imu_simulator.model_parameter_settings.gyroscope.K")
          .as_double_array();
  std::vector<double> gyroCorrTime =
      this->get_parameter(
              "imu_simulator.model_parameter_settings.gyroscope."
              "correlation_time")
          .as_double_array();
  std::vector<double> gyroIntervalTurnOnBias =
      this->get_parameter(
              "imu_simulator.model_parameter_settings.gyroscope.interval_turn_"
              "on_bias")
          .as_double_array();
  std::vector<double> gyroMeasRange =
      this->get_parameter(
              "imu_simulator.model_parameter_settings.gyroscope.measurement_"
              "range")
          .as_double_array();
  std::vector<double> gyroIntervalMisAlignment =
      this->get_parameter(
              "imu_simulator.model_parameter_settings.gyroscope.interval_"
              "misalignment")
          .as_double_array();
  std::vector<double> gyroIntervalNonOrthogonality =
      this->get_parameter(
              "imu_simulator.model_parameter_settings.gyroscope.interval_non_"
              "orthogonality")
          .as_double_array();
  std::vector<double> gyroIntervalScaleFactor =
      this->get_parameter(
              "imu_simulator.model_parameter_settings.gyroscope.interval_scale_"
              "factor")
          .as_double_array();
  std::vector<double> gyroResolution =
      this->get_parameter(
              "imu_simulator.model_parameter_settings.gyroscope.resolution")
          .as_double_array();

  // Assign model parameters to struct accelerometer
  accSimParams.N = doubleVectorToEigenVector(accN);
  accSimParams.B = doubleVectorToEigenVector(accB);
  accSimParams.K = doubleVectorToEigenVector(accK);
  accSimParams.corrTime = doubleVectorToEigenVector(accCorrTime);
  accSimParams.intervalTurnOnBias =
      doubleVectorToEigenVector(accIntervalTurnOnBias);
  accSimParams.measRange = doubleVectorToEigenVector(accMeasRange);
  accSimParams.intervalMisAlignment =
      doubleVectorToEigenVector(accIntervalMisAlignment);
  accSimParams.intervalNonOrthogonality =
      doubleVectorToEigenVector(accIntervalNonOrthogonality);
  accSimParams.intervalScaleFactor =
      doubleVectorToEigenVector(accIntervalScaleFactor);
  accSimParams.resolution = doubleVectorToEigenVector(accResolution);

  // Assign model parameters to struct gyroscope
  gyroSimParams.N = doubleVectorToEigenVector(gyroN);
  gyroSimParams.B = doubleVectorToEigenVector(gyroB);
  gyroSimParams.K = doubleVectorToEigenVector(gyroK);
  gyroSimParams.corrTime = doubleVectorToEigenVector(gyroCorrTime);
  gyroSimParams.intervalTurnOnBias =
      doubleVectorToEigenVector(gyroIntervalTurnOnBias);
  gyroSimParams.measRange = doubleVectorToEigenVector(gyroMeasRange);
  gyroSimParams.intervalMisAlignment =
      doubleVectorToEigenVector(gyroIntervalMisAlignment);
  gyroSimParams.intervalNonOrthogonality =
      doubleVectorToEigenVector(gyroIntervalNonOrthogonality);
  gyroSimParams.intervalScaleFactor =
      doubleVectorToEigenVector(gyroIntervalScaleFactor);
  gyroSimParams.resolution = doubleVectorToEigenVector(gyroResolution);

  // Set IMU simulator parameters
  pImuSimulator_->setImuSimParameters(accSimParams, gyroSimParams);
}

/**
 * @brief Declare and retrieve IMU model enable settings from the parameter server
*/
void ImuSimulatorNode::declareAndRetrieveEnableSettings() {
  // Model enable settings IMU
  ImuModelEnableSettings imuModelEnableSettings;

  // Model enable settings accelerometer
  ModelEnableSettings accModelEnableSettings;

  // Model enable settings gyroscope
  ModelEnableSettings gyroModelEnableSettings;

  // Model enable settings accelerometer
  this->declare_parameter(
      "imu_simulator.model_enable_settings.accelerometer.enable_local_ref_vec",
      rclcpp::PARAMETER_BOOL);
  this->declare_parameter(
      "imu_simulator.model_enable_settings.accelerometer.enable_turn_on_bias",
      rclcpp::PARAMETER_BOOL);
  this->declare_parameter(
      "imu_simulator.model_enable_settings.accelerometer.enable_scaling",
      rclcpp::PARAMETER_BOOL);
  this->declare_parameter(
      "imu_simulator.model_enable_settings.accelerometer.enable_misalignment",
      rclcpp::PARAMETER_BOOL);
  this->declare_parameter(
      "imu_simulator.model_enable_settings.accelerometer.enable_stochastic_"
      "error",
      rclcpp::PARAMETER_BOOL);
  this->declare_parameter(
      "imu_simulator.model_enable_settings.accelerometer.enable_saturation",
      rclcpp::PARAMETER_BOOL);
  this->declare_parameter(
      "imu_simulator.model_enable_settings.accelerometer.enable_quantization",
      rclcpp::PARAMETER_BOOL);

  // Model enable settings gyroscope
  this->declare_parameter(
      "imu_simulator.model_enable_settings.gyroscope.enable_local_ref_vec",
      rclcpp::PARAMETER_BOOL);
  this->declare_parameter(
      "imu_simulator.model_enable_settings.gyroscope.enable_turn_on_bias",
      rclcpp::PARAMETER_BOOL);
  this->declare_parameter(
      "imu_simulator.model_enable_settings.gyroscope.enable_scaling",
      rclcpp::PARAMETER_BOOL);
  this->declare_parameter(
      "imu_simulator.model_enable_settings.gyroscope.enable_misalignment",
      rclcpp::PARAMETER_BOOL);
  this->declare_parameter(
      "imu_simulator.model_enable_settings.gyroscope.enable_stochastic_error",
      rclcpp::PARAMETER_BOOL);
  this->declare_parameter(
      "imu_simulator.model_enable_settings.gyroscope.enable_saturation",
      rclcpp::PARAMETER_BOOL);
  this->declare_parameter(
      "imu_simulator.model_enable_settings.gyroscope.enable_quantization",
      rclcpp::PARAMETER_BOOL);

  // Retrieve model enable settings accelerometer
  bool accEnableLocalRefVec =
      this->get_parameter(
              "imu_simulator.model_enable_settings.accelerometer."
              "enable_local_ref_vec")
          .as_bool();
  bool accEnableTurnOnBias =
      this->get_parameter(
              "imu_simulator.model_enable_settings.accelerometer."
              "enable_turn_on_bias")
          .as_bool();
  bool accEnableScaling =
      this->get_parameter(
              "imu_simulator.model_enable_settings.accelerometer."
              "enable_scaling")
          .as_bool();
  bool accEnableMisAlignment = this->get_parameter(
                                       "imu_simulator.model_enable_settings."
                                       "accelerometer.enable_misalignment")
                                   .as_bool();
  bool accEnableStochasticError =
      this->get_parameter(
              "imu_simulator.model_enable_settings.accelerometer."
              "enable_stochastic_error")
          .as_bool();
  bool accEnableSaturation = this->get_parameter(
                                     "imu_simulator.model_enable_settings."
                                     "accelerometer.enable_saturation")
                                 .as_bool();
  bool accEnableQuantization = this->get_parameter(
                                       "imu_simulator.model_enable_settings."
                                       "accelerometer.enable_quantization")
                                   .as_bool();

  // Retrieve model enable settings gyroscope
  bool gyroEnableLocalRefVec =
      this->get_parameter(
              "imu_simulator.model_enable_settings.gyroscope.enable_local_ref_"
              "vec")
          .as_bool();
  bool gyroEnableTurnOnBias = this->get_parameter(
                                      "imu_simulator.model_enable_settings."
                                      "gyroscope.enable_turn_on_bias")
                                  .as_bool();
  bool gyroEnableScaling = this->get_parameter(
                                   "imu_simulator."
                                   "model_enable_settings."
                                   "gyroscope."
                                   "enable_scaling")
                               .as_bool();
  bool gyroEnableMisAlignment = this->get_parameter(
                                        "imu_simulator.model_enable_settings."
                                        "gyroscope.enable_misalignment")
                                    .as_bool();
  bool gyroEnableStochasticError =
      this->get_parameter(
              "imu_simulator.model_enable_settings."
              "gyroscope.enable_stochastic_error")
          .as_bool();
  bool gyroEnableSaturation = this->get_parameter(
                                      "imu_simulator.model_enable_settings."
                                      "gyroscope.enable_saturation")
                                  .as_bool();
  bool gyroEnableQuantization = this->get_parameter(
                                        "imu_simulator.model_enable_settings."
                                        "gyroscope.enable_quantization")
                                    .as_bool();

  // Assign model enable settings to struct accelerometer
  accModelEnableSettings.enableLocalRefVec = accEnableLocalRefVec;
  accModelEnableSettings.enableTurnOnBias = accEnableTurnOnBias;
  accModelEnableSettings.enableScaling = accEnableScaling;
  accModelEnableSettings.enableMisAlignment = accEnableMisAlignment;
  accModelEnableSettings.enableStochasticError = accEnableStochasticError;
  accModelEnableSettings.enableSaturation = accEnableSaturation;
  accModelEnableSettings.enableQuantization = accEnableQuantization;

  // Assign model enable settings to struct gyroscope
  gyroModelEnableSettings.enableLocalRefVec = gyroEnableLocalRefVec;
  gyroModelEnableSettings.enableTurnOnBias = gyroEnableTurnOnBias;
  gyroModelEnableSettings.enableScaling = gyroEnableScaling;
  gyroModelEnableSettings.enableMisAlignment = gyroEnableMisAlignment;
  gyroModelEnableSettings.enableStochasticError = gyroEnableStochasticError;
  gyroModelEnableSettings.enableSaturation = gyroEnableSaturation;
  gyroModelEnableSettings.enableQuantization = gyroEnableQuantization;

  // Assign model enable settings to struct IMU model enable settings
  imuModelEnableSettings.acc = accModelEnableSettings;
  imuModelEnableSettings.gyro = gyroModelEnableSettings;

  // Set IMU simulator enable settings
  pImuSimulator_->setImuEnableSettings(imuModelEnableSettings);
}

/**
 * @brief Declare and retrieve environmental settings from the parameter server
*/
void ImuSimulatorNode::declareAndRetrieveEnvironmentalSettings() {
  // Geographic position and velocity
  Eigen::Vector3d geoPositionLlh;
  Eigen::Vector3d geoVelocityNed;

  // Declare environmental settings
  this->declare_parameter(
      "imu_simulator.environmental_settings.start_geo_position_llh",
      std::vector<double>{0.0, 0.0, 0.0});
  this->declare_parameter(
      "imu_simulator.environmental_settings.start_geo_velocity_ned",
      rclcpp::PARAMETER_DOUBLE_ARRAY);

  // Retrieve environmental settings
  std::vector<double> geoPosition =
      this->get_parameter(
              "imu_simulator.environmental_settings.start_geo_position_llh")
          .as_double_array();
  std::vector<double> geoVelocity =
      this->get_parameter(
              "imu_simulator.environmental_settings.start_geo_velocity_ned")
          .as_double_array();

  // Convert latitude and longitude to radians
  geoPositionLlh << geoPosition[0] * M_PI / 180.0,
      geoPosition[1] * M_PI / 180.0, geoPosition[2];

  geoVelocityNed = doubleVectorToEigenVector(geoVelocity);

  // Set IMU simulator parameters
  pImuSimulator_->setImuGeoPositionLlh(geoPositionLlh);
  pImuSimulator_->setImuGeoVelocity(geoVelocityNed);
}

/**
 * @brief IMU simulator loop callback function.
 * 
 * This function is called periodically by the timer and publishes the IMU
 * and diagnostic messages. The IMU simulator is updated with ground truth
 * data and the simulated IMU data is published. 
*/
void ImuSimulatorNode::imuSimulatorLoopCallback() {
  // Get current time
  rclcpp::Time currentTimestamp = now();

  // Create diagnostic message
  diagnostic_msgs::msg::DiagnosticStatus diagnosticMsg;

  // Create diagnostic array message
  diagnostic_msgs::msg::DiagnosticArray diagnosticArrayMsg;

  // Read out odometry message
  Eigen::Vector3d v_nb_b_true;
  Eigen::Vector3d w_ib_b_true;
  Eigen::Quaterniond q_ib_true;

  // Check if ground truth odometry message is available
  if (groundTruthOdomMsg_ == nullptr || groundTruthAccelMsg_ == nullptr) {
    // Print STALE diagnostic message when no ground truth messages
    diagnosticMsg.level = diagnostic_msgs::msg::DiagnosticStatus::STALE;
    diagnosticMsg.name = "IMU Simulator";
    diagnosticMsg.message =
        "Waiting for first ground truth odometry and acceleration message!";

    // Add diagnostic message to diagnostic array message
    diagnosticArrayMsg.status.push_back(diagnosticMsg);
    diagnosticArrayMsg.header.stamp = currentTimestamp;

    // Publish the diagnostic array message
    pDiagnosticPublisher_->publish(diagnosticArrayMsg);

    // Reset odometry timeout timer since waiting for first message
    pOdometryTimeOutTimer_->cancel();
    pOdometryTimeOutTimer_->reset();

    if (!calc_acc_from_odom_vel_) {
      // Reset acceleration timeout timer since waiting for first message
      pAccelerationTimeOutTimer_->cancel();
      pAccelerationTimeOutTimer_->reset();
    }

    return;

  } else {
    // Return if this is the first odometry message
    if (!firstLinearVelocityReceived_) {
      // Print STALE diagnostic message when acceleration cannot be calculated
      diagnosticMsg.level = diagnostic_msgs::msg::DiagnosticStatus::STALE;
      diagnosticMsg.name = "IMU Simulator";
      diagnosticMsg.message =
          "Waiting for second linear velocity message to generate "
          "ground truth acceleration input for IMU simulator!";

      // Add diagnostic message to diagnostic array message
      diagnosticArrayMsg.status.push_back(diagnosticMsg);
      diagnosticArrayMsg.header.stamp = currentTimestamp;

      // Publish the diagnostic array message
      pDiagnosticPublisher_->publish(diagnosticArrayMsg);

      // Reset odometry timeout timer since waiting for first message
      pOdometryTimeOutTimer_->cancel();
      pOdometryTimeOutTimer_->reset();

      if (!calc_acc_from_odom_vel_) {
        // Reset acceleration timeout timer since waiting for first message
        pAccelerationTimeOutTimer_->cancel();
        pAccelerationTimeOutTimer_->reset();
      }

      // Assign current time to time stamp of last odometry message
      timeLastOdom_ = currentTimestamp;

      // Set first linear velocity received flag
      firstLinearVelocityReceived_ = true;

      return;
    }

    // Assign ground truth odometry message to IMU simulator inputs
    Eigen::Vector3d v_ib_b_true;
    v_ib_b_true << groundTruthOdomMsg_.get()->twist.twist.linear.x,
        groundTruthOdomMsg_.get()->twist.twist.linear.y,
        groundTruthOdomMsg_.get()->twist.twist.linear.z;

    // Calculate time since last odometry message
    rclcpp::Duration dt = rclcpp::Duration(now() - timeLastOdom_);

    // Assign current time to time stamp of last odometry message
    timeLastOdom_ = currentTimestamp;

    Eigen::Vector3d a_ib_b_true;
    if (calc_acc_from_odom_vel_) {
      // Calculate linear acceleration from odometry linear velocity
      a_ib_b_true = (v_ib_b_true - v_ib_b_prev_) / (dt.nanoseconds() * 1e-9);
    } else {
      // Extract linear acceleration from ground truth acceleration message
      a_ib_b_true(0) = groundTruthAccelMsg_.get()->accel.linear.x;
      a_ib_b_true(1) = groundTruthAccelMsg_.get()->accel.linear.y;
      a_ib_b_true(2) = groundTruthAccelMsg_.get()->accel.linear.z;
    }

    // Assign current velocity to previous velocity
    v_ib_b_prev_ << v_nb_b_true(0), v_nb_b_true(1), v_nb_b_true(2);

    // Assign true angular rate
    w_ib_b_true << groundTruthOdomMsg_.get()->twist.twist.angular.x,
        groundTruthOdomMsg_.get()->twist.twist.angular.y,
        groundTruthOdomMsg_.get()->twist.twist.angular.z;

    // Assign true acceleration to true acceleration message
    geometry_msgs::msg::Vector3 trueAccelerationMsg;

    trueAccelerationMsg.x = a_ib_b_true(0);
    trueAccelerationMsg.y = a_ib_b_true(1);
    trueAccelerationMsg.z = a_ib_b_true(2);

    // Assign true angular rate to true angular rate message
    geometry_msgs::msg::Vector3 trueAngularRateMsg;

    trueAngularRateMsg.x = w_ib_b_true(0);
    trueAngularRateMsg.y = w_ib_b_true(1);
    trueAngularRateMsg.z = w_ib_b_true(2);

    // Publish true acceleration message
    pTrueAccelerationPublisher_->publish(trueAccelerationMsg);

    // Publish true angular rate message
    pTrueAngularRatePublisher_->publish(trueAngularRateMsg);

    // Assign true orientation to quaternion
    q_ib_true.w() = groundTruthOdomMsg_.get()->pose.pose.orientation.w;
    q_ib_true.x() = groundTruthOdomMsg_.get()->pose.pose.orientation.x;
    q_ib_true.y() = groundTruthOdomMsg_.get()->pose.pose.orientation.y;
    q_ib_true.z() = groundTruthOdomMsg_.get()->pose.pose.orientation.z;

    // Generate acceleration measurement
    Eigen::Vector3d f_ib_b_meas =
        pImuSimulator_->generateAccelerationMeasurement(a_ib_b_true, q_ib_true);

    // Generate gyroscope measurement
    Eigen::Vector3d w_ib_b_meas =
        pImuSimulator_->generateGyroscopeMeasurement(w_ib_b_true, q_ib_true);

    // Fill and fill the IMU message
    sensor_msgs::msg::Imu imuMsg;

    imuMsg.header.stamp = currentTimestamp;
    imuMsg.header.frame_id = "imu_link";

    // Fill quaternion and orientation covariance with zeros
    imuMsg.orientation.w = 1.0;
    imuMsg.orientation.x = 0.0;
    imuMsg.orientation.y = 0.0;
    imuMsg.orientation.z = 0.0;

    for (size_t i = 0; i < 9; i++) {
      imuMsg.orientation_covariance[i] = 0.0;
    }

    // Fill acceleration with simulated data
    imuMsg.linear_acceleration.x = f_ib_b_meas(0);
    imuMsg.linear_acceleration.y = f_ib_b_meas(1);
    imuMsg.linear_acceleration.z = f_ib_b_meas(2);

    // Fill acceleration covariance with zeros
    for (size_t i = 0; i < 9; i++) {
      imuMsg.linear_acceleration_covariance[i] = 0.0;
    }

    // Fill angular velocity with simulated data
    imuMsg.angular_velocity.x = w_ib_b_meas(0);
    imuMsg.angular_velocity.y = w_ib_b_meas(1);
    imuMsg.angular_velocity.z = w_ib_b_meas(2);

    // Fill angular velocity covariance with zeros
    for (size_t i = 0; i < 9; i++) {
      imuMsg.angular_velocity_covariance[i] = 0.0;
    }

    // Publish the IMU message
    pImuPublisher_->publish(imuMsg);

    // Define and fill the IMU visualization message
    geometry_msgs::msg::AccelStamped accelStampedMsg;

    accelStampedMsg.header.stamp = currentTimestamp;
    accelStampedMsg.header.frame_id = "imu_link";

    // Fill acceleration with simulated data
    accelStampedMsg.accel.linear.x = f_ib_b_meas(0);
    accelStampedMsg.accel.linear.y = f_ib_b_meas(1);
    accelStampedMsg.accel.linear.z = f_ib_b_meas(2);

    accelStampedMsg.accel.angular.x = w_ib_b_meas(0);
    accelStampedMsg.accel.angular.y = w_ib_b_meas(1);
    accelStampedMsg.accel.angular.z = w_ib_b_meas(2);

    // Publish the visualization message
    pImuAccelStampedPublisher_->publish(accelStampedMsg);

    // Fill the diagnostic message
    diagnosticMsg.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    diagnosticMsg.name = "IMU Simulator";
    diagnosticMsg.message = "IMU simulator running nominal.";
  }

  // Calculate time since last odometry message
  rclcpp::Duration timeSinceLastOdom =
      rclcpp::Duration(currentTimestamp - lastOdomTimestamp_);

  // Calculate time since last acceleration message
  rclcpp::Duration timeSinceLastAccel =
      rclcpp::Duration(currentTimestamp - lastAccelTimestamp_);

  // Check if odometry or acceleration message frequency is too slow
  if (timeSinceLastOdom.seconds() > sampleTime_ && !odometry_timeout_ &&
      timeSinceLastAccel.seconds() > sampleTime_ && !acceleration_timeout_) {
    diagnosticMsg.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    diagnosticMsg.name = "IMU Simulator";
    diagnosticMsg.message =
        "Ground truth odometry or acceleration message frequency is too "
        "slow! "
        "IMU simulator ground truth frequency higher than "
        "odometry/aceleration! Increase odometry/aceleration message "
        "frequency!";
  }

  // Check if odometry or acceleration timeout flag is set
  if (odometry_timeout_ || acceleration_timeout_) {
    diagnosticMsg.level = diagnostic_msgs::msg::DiagnosticStatus::STALE;
    diagnosticMsg.name = "IMU Simulator";
    diagnosticMsg.message =
        "No ground truth odometry or acceleration message received since "
        "than "
        "5 seconds! IMU simulator stalling!";

    // Set first odometry received flag to false since too long time since last
    firstLinearVelocityReceived_ = false;
  }

  // Add diagnostic message to diagnostic array message
  diagnosticArrayMsg.status.push_back(diagnosticMsg);
  diagnosticArrayMsg.header.stamp = currentTimestamp;

  // Publish the diagnostic message
  pDiagnosticPublisher_->publish(diagnosticArrayMsg);
}

/**
 * @brief Odometry callback function.
 * 
 * This function is called when a new odometry message is received. The ground
 * truth odometry message is assigned to the ground truth odometry message
 * member variable of the node class.
 * 
 * @param msg Pointer to the odometry message
*/
void ImuSimulatorNode::odometryCallback(
    const nav_msgs::msg::Odometry::SharedPtr msg) {
  // Reset odometry timeout timer
  pOdometryTimeOutTimer_->cancel();
  pOdometryTimeOutTimer_->reset();

  // Set first odometry received flag
  if (!first_odometry_received_) {
    first_odometry_received_ = true;

    RCLCPP_INFO(get_logger(), "First ground truth odometry message received!");

    // Check if first acceleration has been received
    if (first_acceleration_received_) {
      RCLCPP_INFO(get_logger(), "IMU simulator now running nominal!");
    } else {
      RCLCPP_INFO(get_logger(),
                  "Waiting for first ground truth acceleration message...");
    }
  }

  // Reset odometry timeout flag
  if (odometry_timeout_) {
    odometry_timeout_ = false;

    if (acceleration_timeout_ && !calc_acc_from_odom_vel_) {
      RCLCPP_INFO(get_logger(),
                  "Ground truth odometry message received after timeout! Still "
                  "waiting for ground truth acceleration message...");
    } else {
      RCLCPP_INFO(get_logger(),
                  "Ground truth odometry message received after timeout! IMU "
                  "simulator now running nominal!");
    }
  }

  // Assign ground truth odometry message
  groundTruthOdomMsg_ = msg;

  // Assign last odometry timestamp
  lastOdomTimestamp_ = msg->header.stamp;
}

/**
 * @brief Acceleration callback function.
 * 
 * This function is called when a new acceleration message is received. The ground
 * truth acceleration message is assigned to the ground truth acceleration message
 * member variable of the node class.
 * 
 * @param msg Pointer to the acceleration message
*/
void ImuSimulatorNode::accelerationCallback(
    const geometry_msgs::msg::AccelStamped::SharedPtr msg) {
  // Reset acceleration timeout timer
  pAccelerationTimeOutTimer_->cancel();
  pAccelerationTimeOutTimer_->reset();

  // Set first acceleration received flag
  if (!first_acceleration_received_) {
    first_acceleration_received_ = true;
    RCLCPP_INFO(get_logger(),
                "First ground truth acceleration message received!");

    // Check if first odometry velocity has been received
    if (first_odometry_received_) {
      RCLCPP_INFO(get_logger(), "IMU simulator now running nominal!");
    } else {
      RCLCPP_INFO(get_logger(),
                  "Waiting for first ground truth odometry message...");
    }
  }

  // Reset acceleration timeout flag
  if (acceleration_timeout_ == true) {
    acceleration_timeout_ = false;

    if (odometry_timeout_) {
      RCLCPP_INFO(get_logger(),
                  "Ground truth acceleration message received after timeout! "
                  "Still waiting for ground truth odometry message...");
    } else {
      RCLCPP_INFO(
          get_logger(),
          "Ground truth acceleration message received after timeout! IMU "
          "simulator now running nominal!");
    }
  }

  // Assign ground truth acceleration message
  groundTruthAccelMsg_ = msg;

  // Assign last acceleration timestamp
  lastAccelTimestamp_ = msg->header.stamp;
}

/**
 * @brief Odometry timeout callback function.
 * 
 * This function is called when no ground truth odometry message is received.
 * 
*/
void ImuSimulatorNode::odometryTimeOutCallback() {
  // Set odometry timeout flag
  odometry_timeout_ = true;

  RCLCPP_WARN(get_logger(),
              "No ground truth odometry message since more than 5 "
              "seconds! IMU simulator now starting to stale!");
}

/**
 * @brief Acceleration timeout callback function.
 * 
 * This function is called when no ground truth acceleration message is received.
 * 
*/
void ImuSimulatorNode::accelerationTimeOutCallback() {
  // Set acceleration timeout flag
  acceleration_timeout_ = true;

  RCLCPP_WARN(get_logger(),
              "No ground truth acceleration message since more than 5 "
              "seconds! IMU simulator now starting to stale!");
}

/**
 * @brief Publish static tf2 transformations.
*/
void ImuSimulatorNode::publishTf2Transforms() const {
  // Fill tf2 transform message between base_link and imu_link
  geometry_msgs::msg::TransformStamped tfMsg;
  tfMsg.header.stamp = now();

  tfMsg.header.frame_id = "base_link";
  tfMsg.child_frame_id = "imu_link";

  tfMsg.transform.translation.x = 0.0;
  tfMsg.transform.translation.y = 0.0;
  tfMsg.transform.translation.z = 0.0;

  tfMsg.transform.rotation.w = 1.0;
  tfMsg.transform.rotation.x = 0.0;
  tfMsg.transform.rotation.y = 0.0;
  tfMsg.transform.rotation.z = 0.0;

  pStaticTf2Broadcaster_->sendTransform(tfMsg);
}

/**
 * @brief Helper function to convert a Nx1 vector of doubles to an Eigen vector
 * 
 * @param vec Vector of doubles
 * @return Eigen vector
 */
Eigen::VectorXd ImuSimulatorNode::doubleVectorToEigenVector(
    const std::vector<double>& vec) {
  // Convert vector of doubles to Eigen dynamic-sized vector.
  Eigen::VectorXd eigenVec(vec.size());

  // Fill the Eigen vector with values from the input vector.
  for (std::size_t i = 0; i < vec.size(); ++i) {
    eigenVec[i] = vec[i];
  }

  return eigenVec;
}

}  // namespace imu_simulator

/**
 * @brief Main function of the IMU simulator node.
 * 
 * @param argc Number of command line arguments
 * @param argv Command line arguments
 * 
 * @return int Return value
*/
int main(int argc, char** argv) {
  // Create IMU simulator object
  std::shared_ptr<imu_simulator::ImuSimulator> pImuSimulator =
      std::make_shared<imu_simulator::ImuSimulator>();

  rclcpp::init(argc, argv);
  rclcpp::spin(
      std::make_shared<imu_simulator::ImuSimulatorNode>(pImuSimulator));
  rclcpp::shutdown();

  return 0;
}
