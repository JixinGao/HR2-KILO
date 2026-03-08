#include "parameters.h"

bool is_first_frame = true;
double lidar_end_time = 0.0, first_lidar_time = 0.0;
double last_timestamp_lidar = -1.0, last_timestamp_imu = -1.0, last_timestamp_joint = -1.0;
int pcd_index = 0;
IVoxType::Options ivox_options_;
int ivox_nearby_type = 6;

std::vector<double> extrinT(3, 0.0);
std::vector<double> extrinR(9, 0.0);
std::vector<double> extrinRfl(9, 0.0);
std::string lid_topic, imu_topic, encoder_topic;
bool check_satu = true;
bool space_down_sample = true, publish_odometry_without_downsample = false;
int  init_map_size = 10;
double match_s = 81, satu_acc, satu_gyro;
float  plane_thr = 0.1f;
double filter_size_surf_min = 0.5, filter_size_map_min = 0.5;
bool   imu_en = true;
double laser_point_cov = 0.01, acc_norm;
double vel_cov;
double b_gyr_cov, b_acc_cov, b_pc_cov;
double imu_meas_acc_cov, imu_meas_omg_cov; 
double joint_meas_vel_cov, joint_meas_pos_cov;
double gyr_cov_output = 1000.0, acc_cov_output = 500.0;

int    lidar_type, pcd_save_interval;
std::vector<double> gravity;
bool   runtime_pos_log, pcd_save_en, path_en;
bool   scan_pub_en, scan_body_pub_en;
shared_ptr<Preprocess> p_pre;
shared_ptr<ImuProcess> p_imu;
double time_update_last = 0.0, time_current = 0.0, time_predict_last_const = 0.0;

double lidar_time_inte = 0.1, first_imu_time = 0.0;

double sensor_height = 1.2;
bool HandSet_Blind = false;
bool single_direction = false;
int gait_cut = 5;
double base_height_thres = 0.05;
double height_thres_max  = 0.2;
double Vfov = 30;
double ref_swing_multi = 2.5;
double solediff2model = 0.0;
double max_clearance_ratio = 0.9;

int imu_freq = 100, joint_freq = 1000;

MeasureGroup Measures;
ofstream fout_out;

void readParameters(ros::NodeHandle &nh)
{
  p_pre.reset(new Preprocess());
  p_imu.reset(new ImuProcess());
  nh.param<bool>("check_satu", check_satu, 1);
  nh.param<int>("init_map_size", init_map_size, 100);
  nh.param<bool>("space_down_sample", space_down_sample, 1);
  nh.param<double>("mapping/satu_acc",satu_acc,3.0);
  nh.param<double>("mapping/satu_gyro",satu_gyro,35.0);
  nh.param<double>("mapping/acc_norm",acc_norm,1.0);
  nh.param<float>("mapping/plane_thr", plane_thr, 0.05f);
  nh.param<int>("point_filter_num", p_pre->point_filter_num, 2);
  nh.param<std::string>("common/lid_topic",lid_topic,"/livox/lidar");
  nh.param<std::string>("common/imu_topic", imu_topic,"/livox/imu");
  nh.param<std::string>("common/encoder_topic", encoder_topic,"/bhr_b3/foot_state");
  nh.param<int>("common/imu_freq",imu_freq,100);
  nh.param<int>("common/joint_freq",joint_freq,1000);

  nh.param<double>("filter_size_surf",filter_size_surf_min,0.5);
  nh.param<double>("filter_size_map",filter_size_map_min,0.5);
  nh.param<double>("mapping/lidar_meas_cov",laser_point_cov,0.1);
  nh.param<double>("mapping/vel_cov",vel_cov,20);
  nh.param<double>("mapping/b_gyr_cov",b_gyr_cov,0.0001);
  nh.param<double>("mapping/b_acc_cov",b_acc_cov,0.0001);
  nh.param<double>("mapping/b_pc_cov",b_pc_cov,0.01);
  nh.param<double>("mapping/imu_meas_acc_cov",imu_meas_acc_cov,0.1);
  nh.param<double>("mapping/imu_meas_omg_cov",imu_meas_omg_cov,0.1);
  nh.param<double>("mapping/joint_meas_vel_cov",joint_meas_vel_cov,0.1);
  nh.param<double>("mapping/joint_meas_pos_cov",joint_meas_pos_cov,0.1);
  nh.param<double>("preprocess/blind", p_pre->blind, 1.0);
  nh.param<int>("preprocess/lidar_type", lidar_type, 1);
  nh.param<int>("preprocess/scan_line", p_pre->N_SCANS, 16);
  nh.param<int>("preprocess/scan_rate", p_pre->SCAN_RATE, 10);
  nh.param<int>("preprocess/timestamp_unit", p_pre->time_unit, 1);
  nh.param<double>("mapping/match_s", match_s, 81);
  nh.param<std::vector<double>>("mapping/gravity", gravity, std::vector<double>());
  nh.param<std::vector<double>>("mapping/extrinsic_T", extrinT, std::vector<double>());
  nh.param<std::vector<double>>("mapping/extrinsic_R", extrinR, std::vector<double>());
  nh.param<std::vector<double>>("mapping/extrinsic_Rfl", extrinRfl, std::vector<double>());
  nh.param<bool>("odometry/publish_odometry_without_downsample", publish_odometry_without_downsample, false);
  nh.param<bool>("publish/path_en",path_en, true);
  nh.param<bool>("publish/scan_publish_en",scan_pub_en,1);
  nh.param<bool>("publish/scan_bodyframe_pub_en",scan_body_pub_en,1);
  nh.param<bool>("runtime_pos_log_enable", runtime_pos_log, 0);
  nh.param<bool>("pcd_save/pcd_save_en", pcd_save_en, false);
  nh.param<int>("pcd_save/interval", pcd_save_interval, -1);

  nh.param<double>("mapping/lidar_time_inte",lidar_time_inte,0.1);
  nh.param<double>("mapping/lidar_meas_cov",laser_point_cov,0.1);

  nh.param<float>("mapping/ivox_grid_resolution", ivox_options_.resolution_, 0.2);
  nh.param<int>("ivox_nearby_type", ivox_nearby_type, 18);

  nh.param<double>("robot/sensor_height", sensor_height, 1.2);
  nh.param<bool>("robot/enable_HandleSet", HandSet_Blind, false);
  nh.param<bool>("robot/enable_SingleDir", single_direction, false);
  nh.param<int>("robot/gait_cut", gait_cut, 5);
  nh.param<double>("robot/height_thres_min", base_height_thres, 0.06);
  nh.param<double>("robot/height_thres_max", height_thres_max, 0.2);
  nh.param<double>("robot/vfov_lidar", Vfov, 30);
  nh.param<double>("robot/refswing_multi", ref_swing_multi, 2.5);
  nh.param<double>("robot/solediff2model", solediff2model, 0.0);
  nh.param<double>("robot/max_clearance_ratio", max_clearance_ratio, 0.9);

  if (ivox_nearby_type == 0) {
    ivox_options_.nearby_type_ = IVoxType::NearbyType::CENTER;
  } else if (ivox_nearby_type == 6) {
    ivox_options_.nearby_type_ = IVoxType::NearbyType::NEARBY6;
  } else if (ivox_nearby_type == 18) {
    ivox_options_.nearby_type_ = IVoxType::NearbyType::NEARBY18;
  } else if (ivox_nearby_type == 26) {
    ivox_options_.nearby_type_ = IVoxType::NearbyType::NEARBY26;
  } else {
    ivox_options_.nearby_type_ = IVoxType::NearbyType::NEARBY18;
  }
  p_imu->gravity_ << VEC_FROM_ARRAY(gravity);
}

Eigen::Matrix<double, 3, 1> SO3ToEuler(const SO3 &rot) 
{
  double sy = sqrt(rot(0,0)*rot(0,0) + rot(1,0)*rot(1,0));
  bool singular = sy < 1e-6;
  double x, y, z;
  if(!singular)
  {
    x = atan2(rot(2, 1), rot(2, 2));
    y = atan2(-rot(2, 0), sy);   
    z = atan2(rot(1, 0), rot(0, 0));  
  }
  else
  {    
    x = atan2(-rot(1, 2), rot(1, 1));    
    y = atan2(-rot(2, 0), sy);    
    z = 0;
  }
  Eigen::Matrix<double, 3, 1> ang(x, y, z);
  return ang;
}

void open_file()
{
  fout_out.open(DEBUG_FILE_DIR("mat_out.txt"),ios::out);
  if (fout_out)
      cout << "~~~~"<<ROOT_DIR<<" file opened" << endl;
  else
      cout << "~~~~"<<ROOT_DIR<<" doesn't exist" << endl;
}

void reset_cov(Eigen::Matrix<double, 33, 33> & P_init_output)
{
  P_init_output = MD(33, 33)::Identity() * 0.01;
  P_init_output.block<3, 3>(21, 21) = MD(3,3)::Identity() * 0.0001;
  P_init_output.block<6, 6>(24, 24) = MD(6,6)::Identity() * 0.001;
}