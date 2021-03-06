# imu_vicon_arm_calibration
This package is a GUI for guiding user to collect data from Vicon and IMU sensor readings.  Then enable a virual human forearm
tracking system.

## Requirements
1) ROS hydro.

2) 3 IMU (wrist, elbow and shoulder) message in ROS sensor_msgs::Imu.

3) 3 Vicon (wrist, elbow and shoulder) message in ROS geometry_msgs::TransformStamped.

4) Vicon bridge package.

5) X-sense IMU package (Install from X-Sense_API repository).

6) Myo package (install from Myo_API repository).

## Usage
roslaunch vicon_bridge vicon.launch

roslaunch imu_vicon_arm_calibration demo.launch

After imu-vicon calibration and rviz sucessfully load the frames, then run

rosrun imu_vicon_arm_calibration basic_controls.py  to attach part models for each segments.
