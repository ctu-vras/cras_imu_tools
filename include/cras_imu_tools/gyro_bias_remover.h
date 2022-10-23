#pragma once

/**
 * \file
 * \brief 
 * \author Martin Pecka
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-FileCopyrightText: Czech Technical University in Prague
 */

#include <memory>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/console.h>
#include <ros/publisher.h>
#include <ros/subscriber.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Bool.h>
#include <topic_tools/shape_shifter.h>

#include <cras_cpp_common/nodelet_utils.hpp>

namespace cras
{

enum class BiasObserverState
{
  INITIAL_CALIBRATION,
  MOVING,
  STOPPED_SHORT,
  STOPPED_LONG
};

class GyroBiasRemoverNodelet : public cras::Nodelet
{
protected:
  void onInit() override;
  
  void onImuMsg(const sensor_msgs::ImuConstPtr& msg);
  void onOdomMsg(const nav_msgs::OdometryConstPtr& msg);
  void estimateBias(const sensor_msgs::Imu& msg);

  void produceDiagnostics(diagnostic_updater::DiagnosticStatusWrapper& stat);
  void reportBiasChange();
  
  void speak(const std::string& message, ros::console::levels::Level level);
  
  void onReset(const topic_tools::ShapeShifter&);
  void reset();
  
private:
  std::unique_ptr<cras::DiagnosedPublisher<sensor_msgs::Imu>> imuPub;
  ros::Publisher biasPub;
  ros::Publisher speakInfoPub;
  ros::Publisher speakWarnPub;
  ros::Publisher speakErrPub;
  ros::Publisher stopCommandPub;
  ros::Publisher stopLockPub;
  ros::Subscriber imuSub;
  ros::Subscriber isMovingSub;
  ros::Subscriber resetSub;
  
  geometry_msgs::Vector3Stamped gyroBias;
  geometry_msgs::Twist stopCommand;
  std_msgs::Bool stopLockMsg;
  std_msgs::Bool stopUnlockMsg;
  
  BiasObserverState state {BiasObserverState::INITIAL_CALIBRATION};
  
  ros::Duration initialCalibrationDuration;
  size_t minCalibrationSamples {1000};
  double gyroBiasEstimationRate {0.01};
  bool skipCalibration {false};
  bool shouldProduceDiagnostics {true};
  
  double gyroThreshold {0};
  double cmdVelThreshold {0};
  ros::Duration notMovingDurationThreshold;
  
  ros::Duration notMovingDuration;
  ros::Time lastNotMovingTime;
  ros::Time lastReceiveTime;
  size_t numCalibrationSamples {0};
  ros::Time calibrationStartedTime;
  bool hasOdomMsg {false};
};

}