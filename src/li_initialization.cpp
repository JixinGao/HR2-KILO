#include "li_initialization.h"
bool data_accum_finished = false, data_accum_start = false, online_calib_finish = false, refine_print = false;
int frame_num_init = 0;
double time_lag_IMU_wtr_lidar = 0.0;
bool lose_lid = false;
double timediff_imu_wrt_lidar = 0.0;
mutex mtx_buffer;
sensor_msgs::Imu imu_last, imu_next;
sensor_msgs::JointState encoder_last, encoder_next;
double T1[MAXN], s_plot[MAXN], s_plot2[MAXN], s_plot3[MAXN], s_plot11[MAXN];

condition_variable sig_buffer;
int scan_count = 0;
int wait_num = 0;
std::mutex m_time;
bool lidar_pushed = false, imu_pushed = false, encoder_pushed = false;
double joint_timestamp = 0.0;

std::deque<PointCloudXYZI::Ptr>  lidar_buffer;
std::deque<double>               time_buffer;
std::deque<sensor_msgs::Imu::Ptr> imu_deque;
std::deque<sensor_msgs::JointState::Ptr> encoder_deque;
std::deque<ImuJointData> hybrid_deque;
ImuJointData hybrid_data_next, hybrid_data_last;
std::deque<sensor_msgs::Imu>     imu_sd;
std::deque<FootDist>             footdist_sd;
std::deque<double> imu_var_sd;

std::vector<sensor_msgs::Imu::Ptr> StanceRef_buffer;
std::vector<sensor_msgs::Imu::Ptr> SwingRef_buffer;

bool clear_imu = false;
bool clear_encoder = false;

void standard_pcl_cbk(const sensor_msgs::PointCloud2::ConstPtr &msg) 
{
  // mtx_buffer.lock();
  scan_count ++;
  double preprocess_start_time = omp_get_wtime();
  if (msg->header.stamp.toSec() < last_timestamp_lidar)
  {
    ROS_ERROR("lidar loop back, clear buffer");
    // lidar_buffer.shrink_to_fit();
    // mtx_buffer.unlock();
    // sig_buffer.notify_all();
    return;
  }

  last_timestamp_lidar = msg->header.stamp.toSec();
  PointCloudXYZI::Ptr  ptr(new PointCloudXYZI(20000,1));
  p_pre->process(msg, ptr);
  if (ptr->points.size() > 0)
  {
    lidar_buffer.emplace_back(ptr);
    time_buffer.emplace_back(msg->header.stamp.toSec());
  }
  s_plot11[scan_count] = omp_get_wtime() - preprocess_start_time;
  // mtx_buffer.unlock();
  // sig_buffer.notify_all();
}

void livox_pcl_cbk(const livox_ros_driver::CustomMsg::ConstPtr &msg) 
{
  // mtx_buffer.lock();
  double preprocess_start_time = omp_get_wtime();
  scan_count ++;
  if (msg->header.stamp.toSec() < last_timestamp_lidar)
  {
    ROS_ERROR("lidar loop back, clear buffer");
    // mtx_buffer.unlock();
    // sig_buffer.notify_all();
    return;
    // lidar_buffer.shrink_to_fit();
  }

  last_timestamp_lidar = msg->header.stamp.toSec();    
  PointCloudXYZI::Ptr  ptr(new PointCloudXYZI(10000,1));
  p_pre->process(msg, ptr); 
  if (ptr->points.size() > 0)
  {
    lidar_buffer.emplace_back(ptr);
    time_buffer.emplace_back(msg->header.stamp.toSec());
  }
  
  s_plot11[scan_count] = omp_get_wtime() - preprocess_start_time;
  // mtx_buffer.unlock();
  // sig_buffer.notify_all();
}

void imu_cbk(const sensor_msgs::Imu::ConstPtr &msg_in) 
{
  //mtx_buffer.lock();
  if(clear_imu)
  {
    imu_deque.clear();
    clear_imu = false;
  }
  sensor_msgs::Imu::Ptr msg(new sensor_msgs::Imu(*msg_in));
  msg->header.stamp = ros::Time().fromSec(msg->header.stamp.toSec() - timediff_imu_wrt_lidar - time_lag_IMU_wtr_lidar);
  double timestamp = msg->header.stamp.toSec();

  // printf("time_diff%f, %f, %f\n", last_timestamp_imu - timestamp, last_timestamp_imu, timestamp);
  if (timestamp < last_timestamp_imu)
  {
    ROS_ERROR("imu loop back, clear deque");
    // mtx_buffer.unlock();
    // sig_buffer.notify_all();
    return;
  }
  imu_deque.emplace_back(msg);
  last_timestamp_imu = timestamp;
  //mtx_buffer.unlock();
  //sig_buffer.notify_all();
}

void encoder_cbk(const sensor_msgs::JointState::ConstPtr &msg_in)
{
  //mtx_buffer.lock();
  if(clear_encoder)
  {
    encoder_deque.clear();
    clear_encoder = false;
  }
  sensor_msgs::JointState::Ptr msg(new sensor_msgs::JointState(*msg_in));
  msg->header.stamp = ros::Time().fromSec(msg->header.stamp.toSec());
  joint_timestamp = msg->header.stamp.toSec();
  if(joint_timestamp < last_timestamp_joint)
  {
    ROS_ERROR("encoder loop back, clear deque");
    return;
  }

  /* Get foot in the LiD frame */
  {
    Eigen::Vector3d pos_l, vel_l, pos_r, vel_r;
    pos_l << msg->position[0], msg->position[1], msg->position[2];
    vel_l << msg->velocity[0], msg->velocity[1], msg->velocity[2];
    pos_r << msg->position[3], msg->position[4], msg->position[5];
    vel_r << msg->velocity[3], msg->velocity[4], msg->velocity[5];

    Eigen::Vector3d pos_result_l, pos_result_r, vel_result_l,vel_result_r;
    pos_result_l = Fixed_R_wrt_LiDAR * pos_l;
    pos_result_r = Fixed_R_wrt_LiDAR * pos_r;
    vel_result_l = Fixed_R_wrt_LiDAR * vel_l;
    vel_result_r = Fixed_R_wrt_LiDAR * vel_r;

    msg->position[0] = pos_result_l(0);
    msg->position[1] = pos_result_l(1);
    msg->position[2] = pos_result_l(2);
    msg->position[3] = pos_result_r(0);
    msg->position[4] = pos_result_r(1);
    msg->position[5] = pos_result_r(2);
    msg->velocity[0] = vel_result_l(0);
    msg->velocity[1] = vel_result_l(1);
    msg->velocity[2] = vel_result_l(2);
    msg->velocity[3] = vel_result_r(0);
    msg->velocity[4] = vel_result_r(1);
    msg->velocity[5] = vel_result_r(2);
  }
  encoder_deque.emplace_back(msg);
  last_timestamp_joint = joint_timestamp;

  //mtx_buffer.unlock();
  //sig_buffer.notify_all();
}

bool sync_packages(MeasureGroup &meas)
{
  if (lidar_buffer.empty() || imu_deque.empty())
  {
    return false;
  }
  /*** push a lidar scan ***/
  if(!lidar_pushed)
  {
    lose_lid = false;
    meas.lidar = lidar_buffer.front();
    meas.lidar_beg_time = time_buffer.front();
    if(meas.lidar->points.size() < 1) 
    {
      cout << "lose lidar" << endl;
      lose_lid = true;
    }
    else
    {
      double end_time = meas.lidar->points.back().curvature;
      for (auto pt: meas.lidar->points)
      {
        if (pt.curvature > end_time) { end_time = pt.curvature; }
      }
      lidar_end_time = meas.lidar_beg_time + end_time / double(1000);
      meas.lidar_last_time = lidar_end_time;
    }
    lidar_pushed = true;
  }

  if (!lose_lid && (last_timestamp_imu < lidar_end_time)) { return false; }
  if (lose_lid && last_timestamp_imu < meas.lidar_beg_time + lidar_time_inte) { return false; }
  if (!lose_lid && !imu_pushed)
  { 
    /*** push imu data, and pop from imu buffer ***/
    if (p_imu->imu_need_init_)
    {
      double imu_time = imu_deque.front()->header.stamp.toSec();
      imu_next = *(imu_deque.front());
      meas.imu.shrink_to_fit();
      while (imu_time < lidar_end_time)
      {
        meas.imu.emplace_back(imu_deque.front());
        imu_last = imu_next;
        imu_deque.pop_front();
        if(imu_deque.empty()) break;
        imu_time = imu_deque.front()->header.stamp.toSec();
        imu_next = *(imu_deque.front());
      }
    }
    imu_pushed = true;
  }

  if (lose_lid && !imu_pushed)
  { 
    /*** push imu data, and pop from imu buffer ***/
    if (p_imu->imu_need_init_)
    {
      double imu_time = imu_deque.front()->header.stamp.toSec();
      meas.imu.shrink_to_fit();
      imu_next = *(imu_deque.front());
      while (imu_time < meas.lidar_beg_time + lidar_time_inte)
      {
        meas.imu.emplace_back(imu_deque.front());
        imu_last = imu_next;
        imu_deque.pop_front();
        if(imu_deque.empty()) break;
        imu_time = imu_deque.front()->header.stamp.toSec();
        imu_next = *(imu_deque.front());
      }
    }
    imu_pushed = true;
  }
  
  if(!encoder_pushed)
  {
    if(p_imu->imu_need_init_)
    {
      if(!encoder_deque.empty())
      {
        double encoder_time = encoder_deque.front()->header.stamp.toSec();
        encoder_next = *(encoder_deque.front());
        meas.joint.shrink_to_fit();
        while(encoder_time < lidar_end_time)
        {
          meas.joint.emplace_back(encoder_deque.front());
          encoder_last = encoder_next;
          encoder_deque.pop_front();
          if(encoder_deque.empty()) break;
          encoder_time = encoder_deque.front()->header.stamp.toSec();
          encoder_next = *(encoder_deque.front());
        }
      }
    }
    encoder_pushed = true;
  }
  lidar_buffer.pop_front();
  time_buffer.pop_front();
  lidar_pushed = false;
  imu_pushed = false;
  encoder_pushed = false;
  return true;
}