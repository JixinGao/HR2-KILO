#ifndef Estimator_H
#define Estimator_H

#include "common_lib.h"
#include "parameters.h"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <unordered_set>

extern PointCloudXYZI::Ptr normvec; //(new PointCloudXYZI(100000, 1));
extern std::vector<int> time_seq;
extern PointCloudXYZI::Ptr feats_down_body; //(new PointCloudXYZI());
extern PointCloudXYZI::Ptr feats_down_world; //(new PointCloudXYZI());
extern std::vector<V3D> pbody_list;
extern std::vector<PointVector> Nearest_Points; 
extern std::shared_ptr<IVoxType> ivox_; // localmap in ivox
extern std::vector<float> pointSearchSqDis;
extern bool point_selected_surf[100000]; // = {0};
extern std::vector<M3D> crossmat_list;
extern int effct_feat_num;
extern int k;
extern int idx;
extern V3D angvel_avr, acc_avr, acc_avr_norm;
extern int feats_down_size;
extern V3D Lidar_T_wrt_IMU; //(Zero3d);
extern M3D Lidar_R_wrt_IMU; //(Eye3d);
extern M3D Fixed_R_wrt_LiDAR;
extern V3D pos_contact_body;
extern V3D vel_contact_body;
extern bool contact_moment;
extern V3D CHD_data;

extern double G_m_s2;
extern input_ikfom input_in;

Eigen::Matrix<double, 33, 33> process_noise_cov();

Eigen::Matrix<double, 33, 1> get_f(state_ikfom &s, const input_ikfom &in);

Eigen::Matrix<double, 33, 33> df_dx(state_ikfom &s, const input_ikfom &in);

void h_model_output(state_ikfom &s, Eigen::Matrix3d cov_p, Eigen::Matrix3d cov_R, esekfom::dyn_share_modified<double> &ekfom_data);

void h_model_IMU_output(state_ikfom &s, esekfom::dyn_share_modified<double> &ekfom_data);

void h_model_Joint_output(state_ikfom &s, esekfom::dyn_share_modified<double> &ekfom_data);

void pointBodyToWorld(PointType const * const pi, PointType * const po);

void footIMUtoWorld(PointType const * const pi, PointType * const po);

void setfoot(const sensor_msgs::JointState &foot_state, PointType &footL, PointType &footR);

#endif