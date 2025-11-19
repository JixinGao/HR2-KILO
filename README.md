# HR<sup>2</sup>-KILO

**A High-Rate, Robust, Kinematic-Inertial-LiDAR Odometry for Humanoid Robots**

The paper is accepted by RA-L '25, the dataset have been released and the code will be avaliable soon.

The code implementation is based on [Point-LIO](https://github.com/hku-mars/Point-LIO).

## HR<sup>2</sup>-KILO Dataset

Our dataset can be downloaded from [Google Drive](https://drive.google.com/drive/folders/1O8Mm1s-iD1Jlqc9AnVehYloeVd2RvJ_5?usp=sharing).

We have released a total of 4 sequences for humanoid robot SLAM in rosbag format.

The topic structure is as follows:

```
/g1/joint_state   : sensor_msgs/JointState      # Leg kinematics calculated based on default URDF (without additional rotation)
/g1/joint_state_r : sensor_msgs/JointState      # Leg kinematics in actual LiDAR frame (manually set rotation)
/livox/imu        : sensor_msgs/Imu             # IMU information from MID360
/livox/lidar      : livox_ros_driver2/CustomMsg # LiDAR point cloud from MID360
```

**Notice:**

HR<sup>2</sup>-KILO dataset collected calculated foot information by leg kinematics in LiDAR frame instead of raw joint encoder data.

Messages for `/g1/joint_state` and `/g1/joint_state_r` include foot position and velocity, ordered from left foot (x, y, z) to right foot (x, y, z).