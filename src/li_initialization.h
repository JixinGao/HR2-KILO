#pragma once

#include <common_lib.h>
#include "Estimator.h"
#define MAXN                (720000)

extern bool data_accum_finished, data_accum_start, online_calib_finish, refine_print;
extern int frame_num_init;
extern double time_lag_IMU_wtr_lidar; //, mean_acc_norm = 9.81;

extern double timediff_imu_wrt_lidar;
extern mutex mtx_buffer;
extern condition_variable sig_buffer;
extern int scan_count;
extern int wait_num;
extern std::deque<PointCloudXYZI::Ptr>  lidar_buffer;
extern std::deque<double>               time_buffer;
extern std::deque<sensor_msgs::Imu::Ptr> imu_deque;
extern std::deque<sensor_msgs::JointState::Ptr> encoder_deque;
extern ImuJointData hybrid_data_next, hybrid_data_last;
extern std::deque<ImuJointData> hybrid_deque;
extern std::deque<sensor_msgs::Imu>       imu_sd;
extern std::deque<FootDist>               footdist_sd;
extern std::deque<double> imu_var_sd;

extern std::vector<sensor_msgs::Imu::Ptr> StanceRef_buffer;
extern std::vector<sensor_msgs::Imu::Ptr> SwingRef_buffer;

extern std::mutex m_time;
extern bool lidar_pushed, imu_pushed, encoder_pushed;
extern double joint_timestamp;
extern bool lose_lid;
extern sensor_msgs::Imu imu_last, imu_next;
extern sensor_msgs::JointState encoder_last, encoder_next;
extern double T1[MAXN], s_plot[MAXN], s_plot2[MAXN], s_plot3[MAXN], s_plot11[MAXN];

extern bool clear_imu;
extern bool clear_encoder;

// extern sensor_msgs::Imu::ConstPtr imu_last_ptr;

void standard_pcl_cbk(const sensor_msgs::PointCloud2::ConstPtr &msg); 
void livox_pcl_cbk(const livox_ros_driver::CustomMsg::ConstPtr &msg); 
void imu_cbk(const sensor_msgs::Imu::ConstPtr &msg_in); 
void encoder_cbk(const sensor_msgs::JointState::ConstPtr &msg);
bool sync_packages(MeasureGroup &meas);

// #endif