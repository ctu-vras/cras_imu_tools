/**
 * \file
 * \brief 
 * \author Martin Pecka
 * SPDX-License-Identifier: BSD-3-Clause
 * SPDX-FileCopyrightText: Czech Technical University in Prague
 */

#include <geometry_msgs/Vector3Stamped.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <topic_tools/shape_shifter.h>
#include <std_msgs/String.h>

#include <cras_cpp_common/type_utils/literal_sz.h>

#include <cras_imu_tools/gyro_bias_remover.h>

namespace cras
{

void GyroBiasRemoverNodelet::onInit()
{
  auto nh = this->getNodeHandle();
  auto pnh = this->getPrivateNodeHandle();
  auto params = this->privateParams();
  
  this->initialCalibrationDuration = params->getParam("initial_calibration_duration", ros::Duration(15), "s");
  this->minCalibrationSamples = params->getParam("min_calibration_samples", 1000_sz, "IMU samples");
  this->gyroBiasEstimationRate = params->getParam("bias_estimation_rate", 0.01);
  this->gyroThreshold = params->getParam("gyro_not_moving_threshold", 0.05, "rad/s");
  this->cmdVelThreshold = params->getParam("cmd_vel_threshold", 1e-3, "m/s or rad/s");
  this->notMovingDurationThreshold = params->getParam("not_moving_duration_threshold", ros::Duration(2), "s");
  this->skipCalibration = params->getParam("skip_calibration", false);
  this->shouldProduceDiagnostics = params->getParam("produce_diagnostics", true);
  
  if (this->skipCalibration)
    this->state = BiasObserverState::MOVING;
  
  this->calibrationStartedTime = ros::Time::now();
  
  this->imuPub = this->advertiseDiagnosed<sensor_msgs::Imu>(nh, pnh, "output", "imu_unbiased/data", 100);
  this->biasPub = nh.advertise<geometry_msgs::Vector3Stamped>("imu/gyro_bias", 100, true);
  this->speakInfoPub = nh.advertise<std_msgs::String>("speak/info", 2);
  this->speakWarnPub = nh.advertise<std_msgs::String>("speak/warn", 2);
  this->speakErrPub = nh.advertise<std_msgs::String>("speak/err", 2);

  this->imuSub = nh.subscribe("imu/data", 100, &GyroBiasRemoverNodelet::onImuMsg, this);
  this->isMovingSub = nh.subscribe("odom_cmd_vel", 100, &GyroBiasRemoverNodelet::onOdomMsg, this);
  this->resetSub = nh.subscribe("reset", 1, &GyroBiasRemoverNodelet::onReset, this);
  
  if (this->shouldProduceDiagnostics)
  {
    this->getDiagUpdater().add("status", this, &GyroBiasRemoverNodelet::produceDiagnostics);
    this->startDiagTimer();
  }
  
  this->log->logInfo("IMU calibration has started.");
  this->speak("Calibrating gyro, do not move me.", ros::console::levels::Warn);
}

void GyroBiasRemoverNodelet::onImuMsg(const sensor_msgs::ImuConstPtr& msg)
{
  if (ros::Time::now() + ros::Duration(3) < this->lastReceiveTime)
  {
    this->log->logWarn("ROS time has jumped back, resetting.");
    this->reset();
  }
  this->lastReceiveTime = ros::Time::now();
  
  this->estimateBias(*msg);

  if (this->state == BiasObserverState::INITIAL_CALIBRATION)
    return;
  
  sensor_msgs::Imu unbiased = *msg;
  unbiased.angular_velocity.x -= this->gyroBias.vector.x;
  unbiased.angular_velocity.y -= this->gyroBias.vector.y;
  unbiased.angular_velocity.z -= this->gyroBias.vector.z;

  this->imuPub->publish(unbiased);
}

void GyroBiasRemoverNodelet::onOdomMsg(const nav_msgs::OdometryConstPtr& msg)
{
  this->hasOdomMsg = true;
  this->log->logDebugThrottle(1.0, "State is %lu", this->state);
  
  const auto& v = msg->twist.twist.linear;
  const auto& w = msg->twist.twist.angular;
  
  const auto& th = this->cmdVelThreshold;
  
  const auto isNowMoving = std::abs(v.x) > th || std::abs(v.y) > th || std::abs(v.z) > th || std::abs(w.x) > th
    || std::abs(w.y) > th || std::abs(w.z) > th;
  
  if (isNowMoving && this->state == BiasObserverState::INITIAL_CALIBRATION)
  {
    ROS_ERROR("Robot has moved during IMU calibration!");
    this->speak("Gyro calibration failed, I moved!", ros::console::levels::Error);
  }
  
  if (isNowMoving)
  {
    if (this->state != BiasObserverState::INITIAL_CALIBRATION)
    {
      if (this->state == BiasObserverState::STOPPED_LONG)
        this->reportBiasChange();
      this->state = BiasObserverState::MOVING;
    }
    this->notMovingDuration = {0, 0};
    this->lastNotMovingTime = {0, 0};
    return;
  }
  
  if (this->state == BiasObserverState::MOVING)
  {
    this->state = BiasObserverState::STOPPED_SHORT;
    this->lastNotMovingTime = msg->header.stamp;
    this->notMovingDuration = {0, 0};
    return;
  }

  this->notMovingDuration += msg->header.stamp - this->lastNotMovingTime;
  this->lastNotMovingTime = msg->header.stamp;
  if (this->state != BiasObserverState::INITIAL_CALIBRATION)
  {
    if (this->notMovingDuration > this->notMovingDurationThreshold)
      this->state = BiasObserverState::STOPPED_LONG;
    else
      this->state = BiasObserverState::STOPPED_SHORT;
  }
}

void GyroBiasRemoverNodelet::estimateBias(const sensor_msgs::Imu& msg)
{
  if (this->state != BiasObserverState::INITIAL_CALIBRATION && this->state != BiasObserverState::STOPPED_LONG)
    return;

  const auto& w = msg.angular_velocity;
  auto& bias = this->gyroBias.vector;
  
  if (this->state == BiasObserverState::INITIAL_CALIBRATION)
  {
    bias.x += w.x;
    bias.y += w.y;
    bias.z += w.z;
    this->numCalibrationSamples++;

    const auto calibDuration = msg.header.stamp - this->calibrationStartedTime;
    if (this->numCalibrationSamples >= this->minCalibrationSamples && calibDuration >= this->initialCalibrationDuration)
    {
      bias.x /= static_cast<double>(this->numCalibrationSamples);
      bias.y /= static_cast<double>(this->numCalibrationSamples);
      bias.z /= static_cast<double>(this->numCalibrationSamples);
      
      this->gyroBias.header = msg.header;
      this->biasPub.publish(this->gyroBias);
      
      if (this->lastNotMovingTime == ros::Time(0, 0))
        this->state = BiasObserverState::MOVING;
      else if (this->notMovingDuration >= this->notMovingDurationThreshold)
        this->state = BiasObserverState::STOPPED_LONG;
      else
        this->state = BiasObserverState::STOPPED_SHORT;
      this->log->logWarn("IMU calibration finished.");
      this->speak("Gyros calibrated!", ros::console::levels::Warn);
      this->reportBiasChange();
    }
    else
    {
      this->log->logWarnThrottle(1.0, "IMU is calibrating, do not move the robot.");
    }
  }
  else
  {
    auto th = this->gyroThreshold;

    if (std::abs(w.x - bias.x) > th || std::abs(w.y - bias.y) > th || std::abs(w.z - bias.z) > th)
    {
      this->state = BiasObserverState::MOVING;
      this->lastNotMovingTime = {0, 0};
      this->notMovingDuration = {0, 0};
      this->reportBiasChange();
      return;
    }

    const auto& rate = this->gyroBiasEstimationRate;

    bias.x = (1 - rate) * bias.x + rate * w.x;
    bias.y = (1 - rate) * bias.y + rate * w.y;
    bias.z = (1 - rate) * bias.z + rate * w.z;

    this->gyroBias.header.frame_id = msg.header.frame_id;
    this->gyroBias.header.stamp = msg.header.stamp;

    this->biasPub.publish(this->gyroBias);
  }
}

void GyroBiasRemoverNodelet::reset()
{
  this->state = this->skipCalibration ? BiasObserverState::MOVING : BiasObserverState::INITIAL_CALIBRATION;
  this->numCalibrationSamples = 0;
  this->gyroBias.vector.x = this->gyroBias.vector.y = this->gyroBias.vector.z = 0;
  this->notMovingDuration = {0, 0};
  this->lastNotMovingTime = {0, 0};
  this->calibrationStartedTime = ros::Time::now();
  this->hasOdomMsg = false;
  if (this->shouldProduceDiagnostics)
    this->getDiagUpdater().force_update();
}

void GyroBiasRemoverNodelet::onReset(const topic_tools::ShapeShifter&)
{
  this->reset();
}

void GyroBiasRemoverNodelet::produceDiagnostics(diagnostic_updater::DiagnosticStatusWrapper& stat)
{
  if (this->state == BiasObserverState::INITIAL_CALIBRATION)
    stat.summary(diagnostic_msgs::DiagnosticStatus::WARN, "Initial calibration");
  else if (this->hasOdomMsg)
    stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Running");
  else
    stat.summary(diagnostic_msgs::DiagnosticStatus::WARN, "No odom messages received");

  switch (this->state)
  {
    case BiasObserverState::INITIAL_CALIBRATION:
      stat.add("State", "Initial calibration");
      stat.addf("Num samples", "%lu", this->numCalibrationSamples);
      stat.add("Duration of calibration", cras::to_string(ros::Time::now() - this->calibrationStartedTime));
      break;
    case BiasObserverState::MOVING:
      stat.add("State", "Moving");
      break;
    case BiasObserverState::STOPPED_SHORT:
      stat.add("State", "Just stopped");
      stat.add("Last motion before", cras::to_string(this->notMovingDuration));
      break;
    case BiasObserverState::STOPPED_LONG:
      stat.add("State", "Standing still, calibrating bias");
      stat.add("Last motion before", cras::to_string(this->notMovingDuration));
      break;
  }
}

void GyroBiasRemoverNodelet::reportBiasChange()
{
  this->log->logInfo("Estimated gyro bias is: x=%.6f y=%.6f z=%.6f",
    this->gyroBias.vector.x, this->gyroBias.vector.y, this->gyroBias.vector.z);
}

void GyroBiasRemoverNodelet::speak(const std::string& message, const ros::console::levels::Level level)
{
  std_msgs::String msg;
  msg.data = message;
  
  switch (level)
  {
    case ros::console::levels::Info:
      this->speakInfoPub.publish(msg);
      break;
    case ros::console::levels::Warn:
      this->speakWarnPub.publish(msg);
      break;
    default:
      this->speakErrPub.publish(msg);
      break;
  }
}

}

PLUGINLIB_EXPORT_CLASS(cras::GyroBiasRemoverNodelet, nodelet::Nodelet)