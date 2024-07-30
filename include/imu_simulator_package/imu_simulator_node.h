/*@license BSD-3 https://opensource.org/licenses/BSD-3-Clause
Copyright (c) 2024, Institute of Automatic Control - RWTH Aachen University
Maximilian Nitsch (m.nitsch@irt.rwth-aachen.de)
All rights reserved.
*/

#include "rclcpp/rclcpp.hpp"

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "geometry_msgs/msg/accel_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "tf2_ros/static_transform_broadcaster.h"

#include "imu_simulator.h"

namespace imu_simulator {

class ImuSimulatorNode : public rclcpp::Node {
 public:
  // Constructor with default config file path
  explicit ImuSimulatorNode(std::shared_ptr<ImuSimulator> pImuSimulator);

  // Destructor
  ~ImuSimulatorNode() {}

 private:
  // IMU simulator class object
  std::shared_ptr<ImuSimulator> pImuSimulator_;

  // Ground truth odometry message
  nav_msgs::msg::Odometry::SharedPtr groundTruthOdomMsg_;

  // Ground truth acceleration message
  geometry_msgs::msg::AccelStamped::SharedPtr groundTruthAccelMsg_;

  // Last odometry timestamp
  rclcpp::Time lastOdomTimestamp_;

  // Last acceleration timestamp
  rclcpp::Time lastAccelTimestamp_;

  // IMU message publisher
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pImuPublisher_;

  // IMU visualization publisher
  rclcpp::Publisher<geometry_msgs::msg::AccelStamped>::SharedPtr
      pImuAccelStampedPublisher_;

  // Diagnostic message publisher
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr
      pDiagnosticPublisher_;

  // True acceleration publisher
  rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr
      pTrueAccelerationPublisher_;

  // True angular velocity publisher
  rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr
      pTrueAngularRatePublisher_;

  // Vehicle odometry subscriber
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr pOdometrySubscriber_;

  // Vehicle acceleration subscriber
  rclcpp::Subscription<geometry_msgs::msg::AccelStamped>::SharedPtr
      pAccelerationSubscriber_;

  // Static tf2 broadcaster
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> pStaticTf2Broadcaster_;

  // Diagnostic message
  diagnostic_msgs::msg::DiagnosticStatus diagnosticMsg_;

  // Timers
  rclcpp::TimerBase::SharedPtr pTimer_;
  rclcpp::TimerBase::SharedPtr pOdometryTimeOutTimer_;
  rclcpp::TimerBase::SharedPtr pAccelerationTimeOutTimer_;

  // Sample time
  double sampleTime_;

  // Odometry flags
  bool first_odometry_received_;
  bool odometry_timeout_;

  // Acceleration flags
  bool first_acceleration_received_;
  bool acceleration_timeout_;

  // Flag to indicate if acceleration should be calculated from odometry
  // velocity
  bool calc_acc_from_odom_vel_;

  // Linear velocity
  Eigen::Vector3d v_ib_b_prev_;

  // Flag to indicate if the usable linear velocity message has been received
  bool firstLinearVelocityReceived_;

  // Time of last odometry message
  rclcpp::Time timeLastOdom_;

  // Decleration and retrieval functions for parameters from YAML file
  void declareAndRetrieveGeneralSettings();
  void declareAndRetrieveImuParameters();
  void declareAndRetrieveEnableSettings();
  void declareAndRetrieveEnvironmentalSettings();

  // Timer callback function
  void imuSimulatorLoopCallback();

  // Odometry callback functions
  void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void odometryTimeOutCallback();

  // Acceleration callback functions
  void accelerationCallback(
      const geometry_msgs::msg::AccelStamped::SharedPtr msg);
  void accelerationTimeOutCallback();

  // tf2 publisher
  void publishTf2Transforms() const;

  // Helper functions
  Eigen::VectorXd doubleVectorToEigenVector(const std::vector<double>& vec);
};

}  // namespace imu_simulator