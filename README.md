# cras_imu_tools

Tools for working with IMU messages.

## `cras_imu_tools/gyro_bias_remover` Nodelet and `gyro_bias_remover` Node

Nodelet for measuring and removing gyro bias in IMU messages.

The nodelet expects that the robot (or at least its IMU) is stationary for some time, during which calibration samples are taken.

### Subscribed Topics

- `imu/data` (type `sensor_msgs/Imu`): The biased IMU data.
- `odom_cmd_vel` (type `nav_msgs/Odometry`): An odometry message telling the node whether the robot got some motion command. This can interrupt calibration.
- `reset` (any type): Whenever a message is received on this topic, the nodelet is reset, so for example calibration starts anew.

### Published Topics

- `imu_unbiased/data` (type `sensor_msgs/Imu`): The unbiased IMU data.
- `imu/gyro_bias` (type `geometry_msgs/Vector3Stamped`): The estimated gyro bias values.
- `emergency_stop` (type `std_msgs/Bool`): True is published to this topic during initial calibraiton. You can wire it to a motion disabling system of the robot.
- `emergency_stop/cmd_vel` (type `geometry_msgs/Twist`): Zero velocity commands are published to this topic during initial calibration. You can use it as a high priority motion command to disable motion of the robot.
- `diagnostics`: The bias remover can publish diagnostics if it is configured for it.
- `speak/info` (type `std_msgs/String`): Info-level messages from the calibration. If your robot has an audio output, route these messages to it so that you know the state of the calibration (e.g. via `sound_play` node).
- `speak/warn` (type `std_msgs/String`): Warning-level messages from the calibration. If your robot has an audio output, route these messages to it so that you know the state of the calibration (e.g. via `sound_play` node).
- `speak/err` (type `std_msgs/String`): Error-level messages from the calibration. If your robot has an audio output, route these messages to it so that you know the state of the calibration (e.g. via `sound_play` node).


### Parameters

- `~initial_calibration_duration` (float, default 15.0 s): Duration of the initial calibration procedure when the robot has to be stationary.
- `~min_calibration_samples` (uint, default 1000): The minimum number of IMU messages to collect during initial calibration.
- `~bias_estimation_rate` (float, default 0.01): Rate of change of the estimated bias. This is a parameter of a low-pass filter.
- `~gyro_not_moving_threshold` (float, default 0.05 rad/s): Values of gyro readings that can be treated as stationary. It is important that the bias is smaller than this value, otherwise the calibration will think that the robot was moving during sample collection.
- `~cmd_vel_threshold` (float, default 0.001 m/s or rad/s): Values of motion commands that can be treated as stationary.
- `~not_moving_duration_threshold` (float, default 2.0 s): After how many seconds online recalibration should be started if the robot was stationary for the whole time.
- `~skip_calibration` (bool, default false): If true, the initial calibration will not be performed and the node will right away start working with zero estimated bias.
- `~produce_diagnostics` (bool, default true): If true, the node will produce diagnostic messages.
