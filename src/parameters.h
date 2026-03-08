// #ifndef PARAM_H
// #define PARAM_H
#pragma once
#include <ros/ros.h>
#include <Eigen/Eigen>
#include <Eigen/Core>
#include <cstring>
#include "preprocess.h"
#include "IMU_Processing.h"
#include <sensor_msgs/NavSatFix.h>
#include <livox_ros_driver/CustomMsg.h>
#include <sensor_msgs/PointCloud2.h>
#include <mutex>
#include <omp.h>
#include <math.h>
#include <thread>
#include <fstream>
#include <csignal>
#include <unistd.h>
#include <ivox/ivox3d.h>
#include <Python.h>
#include <condition_variable>
#include <sensor_msgs/Imu.h>
#include <pcl/common/transforms.h>
#include <geometry_msgs/Vector3.h>

// #define IVOX_NODE_TYPE_PHC

#ifdef IVOX_NODE_TYPE_PHC
    using IVoxType = faster_lio::IVox<3, faster_lio::IVoxNodeType::PHC, PointType>;
#else
    using IVoxType = faster_lio::IVox<3, faster_lio::IVoxNodeType::DEFAULT, PointType>;
#endif

extern bool is_first_frame;
extern double lidar_end_time, first_lidar_time;
extern double last_timestamp_lidar, last_timestamp_imu, last_timestamp_joint;
extern int pcd_index;
extern IVoxType::Options ivox_options_;
extern int ivox_nearby_type;
extern std::string lid_topic, imu_topic, encoder_topic;
extern int imu_freq, joint_freq;

extern bool check_satu;
extern bool space_down_sample;
extern bool publish_odometry_without_downsample;
extern int  init_map_size;
extern double match_s, satu_acc, satu_gyro;
extern float  plane_thr;
extern double filter_size_surf_min, filter_size_map_min;
extern bool   imu_en;
extern double laser_point_cov, acc_norm;
extern double vel_cov;
extern double gyr_cov_output, acc_cov_output, b_gyr_cov, b_acc_cov, b_pc_cov;
extern double imu_meas_acc_cov, imu_meas_omg_cov, joint_meas_vel_cov, joint_meas_pos_cov; 
extern int    lidar_type, pcd_save_interval;
extern std::vector<double> gravity;
extern bool   runtime_pos_log, pcd_save_en, path_en;
extern bool   scan_pub_en, scan_body_pub_en;
extern shared_ptr<Preprocess> p_pre;
extern shared_ptr<ImuProcess> p_imu;
extern bool is_first_frame;

extern std::vector<double> extrinT;
extern std::vector<double> extrinR;
extern std::vector<double> extrinRfl;
extern double lidar_time_inte, first_imu_time;
extern double time_update_last, time_current, time_predict_last_const;
extern double sensor_height;
extern bool HandSet_Blind;
extern bool single_direction;
extern int gait_cut;
extern double base_height_thres;
extern double height_thres_max;
extern double Vfov;
extern double ref_swing_multi;
extern double solediff2model;
extern double max_clearance_ratio;

extern MeasureGroup Measures;

extern ofstream fout_out;
void readParameters(ros::NodeHandle &n);
void open_file();
Eigen::Matrix<double, 3, 1> SO3ToEuler(const SO3 &orient);
void reset_cov(Eigen::Matrix<double, 33, 33> & P_init_output);