#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include "li_initialization.h"
#include <malloc.h>

using namespace std;     

#define PUBFRAME_PERIOD     (20)

const float MOV_THRESHOLD = 1.5f;

int time_log_counter = 0;

bool init_map = false, flg_first_scan = true;

// Time Log Variables
double match_time = 0, solve_time = 0, propag_time = 0, update_time = 0;

bool  flg_reset = false, flg_exit = false;

//surf feature in map
PointCloudXYZI::Ptr feats_undistort(new PointCloudXYZI());
PointCloudXYZI::Ptr feats_down_body_space(new PointCloudXYZI());
PointCloudXYZI::Ptr init_feats_world(new PointCloudXYZI());
std::deque<PointCloudXYZI::Ptr> depth_feats_world;
pcl::VoxelGrid<PointType> downSizeFilterSurf;
pcl::VoxelGrid<PointType> downSizeFilterMap;

V3D euler_cur;
nav_msgs::Path path;
nav_msgs::Odometry odomAftMapped;
geometry_msgs::PoseStamped msg_body_pose;

ros::Publisher pub_footdist_hyp;                //test
ros::Publisher pub_IMUVar;                      //test
ros::Publisher pub_pc;
std::vector<float> accum_pointSearchDis;
bool have_hypPlane = false;
VF(4) pabcd_hypPlane;
VF(4) pabcd_realPlane;
VF(4) pre_pabcd_realPlane = Eigen::Vector4f::Zero();

double prev_realPlane_norZ;
M3D Fixed_R_wrt_Body = M3D::Identity();

/** detect contact **/
double algor_beg_time = 0.0;
int gait_cycle = 0;
bool   flg_have_GaitCycle = false;
bool   flg_recording_gait = false;
bool   flg_have_change = true;
bool   flg_IMUchanged = false;
double max_footdist = 0.0;
double beg_swing = 0.0, end_swing = 0.0;
double ref_standing = 0.0, ref_swing = 0.0;
bool flg_footl_stat = true, flg_footr_stat = true;
double last_footl_dist = 0.0, last_footr_dist = 0.0;
int contact_count = 0;
int contact_change_num = 0;
std::vector<Eigen::Matrix<double,6,1>> InitFootW;
ros::Publisher pub_Contact;
int _count  = 0;
bool need_hypPlane = true;
int use_PrvePlane_count = 0;


enum ContactEvent
{
  NONE = 0,
  LeftFoot,
  RightFoot,
  BothFeet
};
ContactEvent flg_contact, last_flg_contact;
ContactEvent prev_flg_contact, prev_phase_flg_contact;

template<typename T>
sensor_msgs::PointCloud2 cloud2msg(pcl::PointCloud<T> cloud, const ros::Time& stamp, std::string frame_id = "map") {
  sensor_msgs::PointCloud2 cloud_ROS;
  pcl::toROSMsg(cloud, cloud_ROS);
  cloud_ROS.header.stamp = stamp;
  cloud_ROS.header.frame_id = frame_id;
  return cloud_ROS;
}

void SigHandle(int sig)
{
  flg_exit = true;
  ROS_WARN("catch sig %d", sig);
  sig_buffer.notify_all();
}

void pointBodyLidarToIMU(PointType const * const pi, PointType * const po)
{
  V3D p_body_lidar(pi->x, pi->y, pi->z);
  V3D p_body_imu;
  p_body_imu = Lidar_R_wrt_IMU * p_body_lidar + Lidar_T_wrt_IMU;
  po->x = p_body_imu(0);
  po->y = p_body_imu(1);
  po->z = p_body_imu(2);
  po->intensity = pi->intensity;
}

void MapIncremental() {
  PointVector points_to_add;
  int cur_pts = feats_down_world->size();
  points_to_add.reserve(cur_pts);

  for (size_t i = 0; i < cur_pts; ++i) {
    /* decide if need add to map */
    PointType &point_world = feats_down_world->points[i];
    if (!Nearest_Points[i].empty()) 
    {
      const PointVector &points_near = Nearest_Points[i];
      Eigen::Vector3f center = ((point_world.getVector3fMap() / filter_size_map_min).array().floor() + 0.5) * filter_size_map_min;
      bool need_add = true;
      for (int readd_i = 0; readd_i < points_near.size(); readd_i++) {
        Eigen::Vector3f dis_2_center = points_near[readd_i].getVector3fMap() - center;
        if (fabs(dis_2_center.x()) < 0.5 * filter_size_map_min &&
            fabs(dis_2_center.y()) < 0.5 * filter_size_map_min &&
            fabs(dis_2_center.z()) < 0.5 * filter_size_map_min) {
            need_add = false;
            break;
        }
      }
      if (need_add) {
        points_to_add.emplace_back(point_world);
      }
    } 
    else {points_to_add.emplace_back(point_world);}
  }
  ivox_->AddPoints(points_to_add);
}

void K_means(PointVector &search_result, std::vector<float> &pointSearchDis,  int max_iter)
{
  if(search_result.size() < 5)  return;
  std::vector<float> cluster1, cluster2;
  std::vector<bool> check_outliers(search_result.size(), false);
  float seed_1 = search_result[0].z;
  float seed_2 = search_result[1].z;
  for (int i = 0; i < max_iter; i++)
  {
    float sum1 = 0;
    float sum2 = 0;
    float seed_1_prve = seed_1;
    float seed_2_prve = seed_2;
    cluster1.clear();
    cluster2.clear();

    for (int j = 0; j < search_result.size(); j++)
    { 
      float dis2seed_1 = abs(search_result[j].z - seed_1);
      float dis2seed_2 = abs(search_result[j].z - seed_2);
      if(dis2seed_1 < dis2seed_2)
      {
        cluster1.emplace_back(j);
        sum1 += search_result[j].z;
      }
      else
      {
        cluster2.emplace_back(j);
        sum2 += search_result[j].z;
      }   
    }
    if(cluster1.size() > 0) seed_1 = sum1 / cluster1.size();
    if(cluster2.size() > 0) seed_2 = sum2 / cluster2.size();
    if((fabs(seed_1 - seed_1_prve) < 1e-6 && fabs(seed_2 - seed_2_prve) < 1e-6) || (i == max_iter-1)) break;
  }

  if(cluster1.size() < cluster2.size())
  {
    for (int i = 0; i < cluster1.size(); i++)
    {
      check_outliers[cluster1[i]] = true;
    }
  }
  else
  {
    for (int i = 0; i < cluster2.size(); i++)
    {
      check_outliers[cluster2[i]] = true;
    }
  }
  for (int  i = search_result.size() -1; i >= 0; --i)
  {
    if(check_outliers[i])
    {
      pointSearchDis.erase(pointSearchDis.begin() + i);
      search_result.erase(search_result.begin() + i);
    }
  }
}

bool MeanFootHypPlane(Eigen::Matrix<double, 6, 1> curr_foot_w, std::vector<PointType> &mean_foot_w)
{
  if (InitFootW.size() < 100)
  {
    InitFootW.push_back(curr_foot_w);
    return false;
  }
  else if (InitFootW.size() == 100)
  {
    Eigen::Matrix<double, 6, 1> tmp;
    tmp << 0.0, 0.0, 0.0,  0.0, 0.0, 0.0;
    for (int i = 0; i < InitFootW.size(); i++)
    {
      tmp += InitFootW[i];
    }
    tmp = tmp / InitFootW.size();
    PointType footL, footR;
    footL.x = tmp(0); footL.y = tmp(1); footL.z = tmp(2);
    footR.x = tmp(3); footR.y = tmp(4); footR.z = tmp(5);
    mean_foot_w.clear();
    mean_foot_w.push_back(footL);
    mean_foot_w.push_back(footR);
    return true;
  }
  else {return true;}
}

Eigen::Vector2d CalDist_Foot2Plane(const sensor_msgs::JointState &foot_state)
{
  Eigen::Vector2d dist_foot = Eigen::Vector2d::Zero();
  PointType footL_in_body, footR_in_body;
  PointType footL_in_world, footR_in_world, footM_in_world;
  setfoot(foot_state, footL_in_body, footR_in_body);
  if(footL_in_body.z < 0)
  {
    footL_in_body.z = footL_in_body.z - solediff2model;
    footR_in_body.z = footR_in_body.z - solediff2model;
  }
  else
  {
    footL_in_body.z = footL_in_body.z + solediff2model;
    footR_in_body.z = footR_in_body.z + solediff2model;
  }
  footIMUtoWorld(&footL_in_body, &footL_in_world);
  footIMUtoWorld(&footR_in_body, &footR_in_world);
  footM_in_world.x = (footL_in_world.x + footR_in_world.x)/2;
  footM_in_world.y = (footL_in_world.y + footR_in_world.y)/2;
  footM_in_world.z = (footL_in_world.z + footR_in_world.z)/2;
  Eigen::Matrix<double, 6, 1> curr_foot_w;
  curr_foot_w << footL_in_world.x, footL_in_world.y , footL_in_world.z , footR_in_world.x, footR_in_world.y, footR_in_world.z;
  std::vector<PointType> mean_foot_w;
  mean_foot_w.push_back(footL_in_world); mean_foot_w.push_back(footR_in_world);
  if (!MeanFootHypPlane(curr_foot_w, mean_foot_w)) {return dist_foot;}

  PointVector search_result_check;
  std::vector<float> pointSearchDis_check;
  search_result_check.clear(); pointSearchDis_check.clear();
  ivox_->GetGroundClosestPoint(footM_in_world, search_result_check, pointSearchDis_check, 10);
  double averdist_foot = 0.0;

  for (int i = 0; i < pointSearchDis_check.size(); i++)
  {
    averdist_foot += (pointSearchDis_check[i] - averdist_foot) / (i+1);
  }
  if(need_hypPlane == true)	//init need hypo plane
  {
		// dist (from nearest points to the bipedal center) > dist (from left/right foot to the bipedal center)
    if (averdist_foot > ((footM_in_world.x-footL_in_world.x)*(footM_in_world.x-footL_in_world.x) + (footM_in_world.y-footL_in_world.y)*(footM_in_world.y-footL_in_world.y) + (footM_in_world.z-footL_in_world.z)*(footM_in_world.z-footL_in_world.z))
     || averdist_foot > ((footM_in_world.x-footR_in_world.x)*(footM_in_world.x-footR_in_world.x) + (footM_in_world.x-footR_in_world.x)*(footM_in_world.x-footR_in_world.x) + (footM_in_world.z-footR_in_world.z)*(footM_in_world.z-footR_in_world.z)))
      need_hypPlane = true;   // have points inside feet
    else
      need_hypPlane = false;  // no points inside
  }
  if(!have_hypPlane)   				// *initial phase, no hypo plane*
  {
    PointVector search_result, search_result_forw, search_result_back;
    std::vector<float> pointSearchDis;
    search_result.clear(); pointSearchDis.clear();

		// get blind dist
    ivox_->GetGroundClosestPoint(footM_in_world, search_result, pointSearchDis, 0.3, 100); // [adjustable] (0.1, 0.3, 0.5)
    for (int i = 0; i < pointSearchDis.size(); i++){
      accum_pointSearchDis.emplace_back(pointSearchDis[i]);
    }
    if(accum_pointSearchDis.size() > 5 || HandSet_Blind == true)
    {
      double averdist_foot2blind = 0.0;
      if(HandSet_Blind != true)
      {
        for (int i = 0; i < accum_pointSearchDis.size(); i++){   
          averdist_foot2blind += (accum_pointSearchDis[i] - averdist_foot2blind) / (i+1);
        }
      }
      else
        averdist_foot2blind = std::pow(sensor_height / tan(Vfov*PI_M/180), 2);
      
			// get extension points
      V3D footM_in_world_vec(footM_in_world.x, footM_in_world.y, footM_in_world.z);
      footM_in_world_vec = Fixed_R_wrt_Body.transpose() * footM_in_world_vec ;
      PointType forw_hypBound = footM_in_world;
      PointType back_hypBound = footM_in_world;
      V3D footForw_in_fixed_vec(footM_in_world_vec(0) + sqrt(averdist_foot2blind), footM_in_world_vec(1), footM_in_world_vec(2));
      V3D footBack_in_fixed_vec(footM_in_world_vec(0) - sqrt(averdist_foot2blind), footM_in_world_vec(1), footM_in_world_vec(2));
      footForw_in_fixed_vec = Fixed_R_wrt_Body * footForw_in_fixed_vec;
      footBack_in_fixed_vec = Fixed_R_wrt_Body * footBack_in_fixed_vec;
      forw_hypBound.x = footForw_in_fixed_vec(0);    forw_hypBound.y = footForw_in_fixed_vec(1);    forw_hypBound.z = footForw_in_fixed_vec(2);
      back_hypBound.x = footBack_in_fixed_vec(0);    back_hypBound.y = footBack_in_fixed_vec(1);    back_hypBound.z = footBack_in_fixed_vec(2); 

      std::vector<float> pointSearchDis_forw, pointSearchDis_back;
      ivox_->GetGroundClosestPoint(forw_hypBound, search_result_forw, pointSearchDis_forw, 0.2, 10);
      ivox_->GetGroundClosestPoint(back_hypBound, search_result_back, pointSearchDis_back, 0.2, 10);
      
      if(search_result_forw.empty() && search_result_back.empty()) return dist_foot;
      std::vector<std::pair<PointType, float>> searchforw_pairs, searchback_pairs;
      for (int i = 0; i < pointSearchDis_forw.size(); i++){
        searchforw_pairs.emplace_back(search_result_forw[i], pointSearchDis_forw[i]);
      }
      for (int j = 0; j < pointSearchDis_back.size(); j++){
        searchback_pairs.emplace_back(search_result_back[j], pointSearchDis_back[j]);
      }
      sort(searchforw_pairs.begin(), searchforw_pairs.end(), dist_list); 
      sort(searchback_pairs.begin(), searchback_pairs.end(), dist_list); 
      
			// get hypo plane
      if(single_direction)
      {
        if (!searchforw_pairs.empty() || !searchback_pairs.empty()){
          pabcd_hypPlane.setZero();
          PointVector point_hypPlane;
          if(!searchforw_pairs.empty())
            point_hypPlane = {mean_foot_w[0], mean_foot_w[1], searchforw_pairs.begin()->first};
          else
            point_hypPlane = {mean_foot_w[0], mean_foot_w[1], searchback_pairs.begin()->first};

          if(esti_plane(pabcd_hypPlane, point_hypPlane, 0.1f, point_hypPlane.size())) have_hypPlane = true;
        }
      }
      else
      {
        if (!searchforw_pairs.empty() && !searchback_pairs.empty())
        {
          pabcd_hypPlane.setZero();
          PointVector point_hypPlane = {
            footL_in_world, footR_in_world, searchforw_pairs.begin()->first, searchback_pairs.begin()->first
          };
          if(esti_plane(pabcd_hypPlane, point_hypPlane, 0.1f, point_hypPlane.size()))	have_hypPlane = true;
        }
      }
    }
    else  std::cout << "search init point < 3, accum_points = " << accum_pointSearchDis.size() << std::endl;
  }

  /* cal the foot clearance */
  bool en_estiplane = false;
  if(!need_hypPlane)
  {
    double left_foot_z = foot_state.position[2];
    double righ_foot_z = foot_state.position[5];
    PointType ref_foot_body, ref_foot_world;
    if(abs(left_foot_z) > abs(righ_foot_z))
    {
      ref_foot_body.x = foot_state.position[0];
      ref_foot_body.y = foot_state.position[1];
      ref_foot_body.z = foot_state.position[2];
    }
    else
    {
      ref_foot_body.x = foot_state.position[3];
      ref_foot_body.y = foot_state.position[4];
      ref_foot_body.z = foot_state.position[5];
    }
    if(ref_foot_body.z < 0)	ref_foot_body.z = ref_foot_body.z - solediff2model;
    else										ref_foot_body.z = ref_foot_body.z + solediff2model;
    footIMUtoWorld(&ref_foot_body, &ref_foot_world);
    PointVector search_result, near_points;
    std::vector<float> pointSearchDis;
    ivox_->GetGroundClosestPoint(ref_foot_world, search_result, pointSearchDis, 35);
    K_means(search_result, pointSearchDis, 3);	// remove outliers
    for (int j = 0; j < pointSearchDis.size(); j++){
      if(pointSearchDis[j] < height_thres_max)	near_points.push_back(search_result[j]);
    }

    if(near_points.size() < 5) {
      if(pre_pabcd_realPlane != Eigen::Vector4f::Zero()){
        pabcd_realPlane = pre_pabcd_realPlane;
        use_PrvePlane_count++;
        if(use_PrvePlane_count > 15)  std::cout << "ground plane warning! using prev plane too long: " << use_PrvePlane_count << std::endl;  
      }
      else  return dist_foot;
    }
    else{
      use_PrvePlane_count = 0;
      pabcd_realPlane.setZero();
      en_estiplane = esti_plane_real(pabcd_realPlane, near_points, 0.1f, near_points.size(), 2);
    }

    if(abs(pabcd_realPlane(2)) < 0.95)   //[todo] cal in world frame.
    {
      if(pre_pabcd_realPlane != Eigen::Vector4f::Zero())
        pabcd_realPlane = pre_pabcd_realPlane;
    }
    else  pre_pabcd_realPlane = pabcd_realPlane;
  }

  sensor_msgs::Imu footdist_msg; 
  footdist_msg.header.stamp = ros::Time(time_current);
  footdist_msg.header.frame_id = "camera_init";

  for(int i = 0; i < 6; i = i+3)
  {
    PointType foot_in_body, foot_in_world;
    foot_in_body.x = foot_state.position[i];
    foot_in_body.y = foot_state.position[i+1];
    foot_in_body.z = foot_state.position[i+2];
    if(foot_in_body.z < 0)  foot_in_body.z = foot_in_body.z - solediff2model;
    else  foot_in_body.z = foot_in_body.z + solediff2model;
    footIMUtoWorld(&foot_in_body, &foot_in_world);

    if(need_hypPlane && have_hypPlane)      // 1  1: need hypo plane, have hypo plane
    {
      double dist_foot_ = abs(pabcd_hypPlane(0)*foot_in_world.x + pabcd_hypPlane(1)*foot_in_world.y + pabcd_hypPlane(2)*foot_in_world.z + pabcd_hypPlane(3));
      if(i == 0)	dist_foot(0) = dist_foot_;
      if(i == 3)	dist_foot(1) = dist_foot_;
      if(1)       //for plot
      {
        if(i == 0)	footdist_msg.linear_acceleration.x = dist_foot_;
        if(i == 3)	footdist_msg.linear_acceleration.y = dist_foot_;
      }
    }
    else if(!need_hypPlane)                 // 0  1 / 0  0
    {
      if(en_estiplane)
      {
        double dist_foot_ = abs(pabcd_realPlane(0)*foot_in_world.x + pabcd_realPlane(1)*foot_in_world.y + pabcd_realPlane(2)*foot_in_world.z + pabcd_realPlane(3));
        if(i == 0)	dist_foot(0) = dist_foot_;
        if(i == 3)	dist_foot(1) = dist_foot_;
        if(i == 0)	footdist_msg.linear_acceleration.x = dist_foot_;
        if(i == 3)	footdist_msg.linear_acceleration.y = dist_foot_;
      }
    }
    else  std::cout << "Wait Init HpyPlane ..." << std::endl;
  }
  pub_footdist_hyp.publish(footdist_msg);
  return dist_foot;
}

bool Get_GaitCycle(Eigen::Vector2d &dist_foot2plane, double timestamp, int &gait_cycle)
{
  if((omp_get_wtime() - algor_beg_time) < 2.0 )	return false;
  if(flg_have_GaitCycle)	return true;

  FootDist footdist_;
  if(!flg_recording_gait)
  {
    if(dist_foot2plane(0) > base_height_thres)
    {
      flg_recording_gait = true;
      footdist_.l_footdist = dist_foot2plane(0);
      footdist_.r_footdist = dist_foot2plane(1);
      footdist_.timestamp  = timestamp;
      footdist_sd.emplace_back(footdist_);
      max_footdist = dist_foot2plane(0);
    }
    return false;
  }
  else
  {
    if(dist_foot2plane(0) >= max_clearance_ratio * max_footdist)
    {
      if(footdist_sd.size() > joint_freq*0.1)
      {
        footdist_.l_footdist = dist_foot2plane(0);
        footdist_.r_footdist = dist_foot2plane(1);
        footdist_.timestamp  = timestamp;
        footdist_sd.emplace_back(footdist_);
        gait_cycle = footdist_sd.size();
        flg_recording_gait = false;
        flg_have_GaitCycle = true;

        //cal swing time
        double mindif_footdist_1 = abs(footdist_sd.front().l_footdist-footdist_sd.front().r_footdist);
        while(footdist_sd.size() > gait_cycle/2)
        {
          if(abs(footdist_sd.front().l_footdist-footdist_sd.front().r_footdist) < mindif_footdist_1)
          {
            mindif_footdist_1 = abs(footdist_sd.front().l_footdist-footdist_sd.front().r_footdist);
            beg_swing = footdist_sd.front().timestamp;
          }
          footdist_sd.pop_front();
        }
        double mindif_footdist_2 = abs(footdist_sd.front().l_footdist-footdist_sd.front().r_footdist);
        while(!footdist_sd.empty())
        {
          if(abs(footdist_sd.front().l_footdist-footdist_sd.front().r_footdist) < mindif_footdist_2)
          {
            mindif_footdist_2 = abs(footdist_sd.front().l_footdist-footdist_sd.front().r_footdist);
            end_swing = footdist_sd.front().timestamp;
          }
          footdist_sd.pop_front();
        }
        std::cout << "\033[1;34mget gait_cycle = " << gait_cycle << "\033[0m" << std::endl;
      }
      else
      {
        if(dist_foot2plane(0) > max_footdist)
        {
          footdist_sd.clear();
          footdist_.l_footdist = dist_foot2plane(0);
          footdist_.r_footdist = dist_foot2plane(1);
          footdist_.timestamp  = timestamp;
          footdist_sd.emplace_back(footdist_);
          max_footdist = dist_foot2plane(0);
        }
        else
        {
          footdist_.l_footdist = dist_foot2plane(0);
          footdist_.r_footdist = dist_foot2plane(1);
          footdist_.timestamp  = timestamp;
          footdist_sd.emplace_back(footdist_);
        }
      }
    }
    else
    {
      footdist_.l_footdist = dist_foot2plane(0);
      footdist_.r_footdist = dist_foot2plane(1);
      footdist_.timestamp  = timestamp;
      footdist_sd.emplace_back(footdist_);
    }
  }
  //std::cout << "size = " << footdist_sd.size() << std::endl;
  return flg_have_GaitCycle;
}

double CalVar(std::deque<sensor_msgs::Imu> &imu_sd)
{
  double aver_x = 0.0, aver_y = 0.0, aver_z = 0.0;
  double ref_standing_x = 0.0, ref_standing_y = 0.0, ref_standing_z = 0.0;
  double var = 0.0;
  for (int i = 0; i < imu_sd.size(); i++) 
  {
    aver_x += (imu_sd[i].linear_acceleration.x * 9.810 - aver_x) / (i + 1);
    ref_standing_x += (imu_sd[i].linear_acceleration.x * 9.810 - aver_x)*(imu_sd[i].linear_acceleration.x * 9.810 - aver_x);
    ref_standing_x = 0.0;

    aver_y += (imu_sd[i].linear_acceleration.y * 9.810 - aver_y) / (i + 1);
    ref_standing_y += (imu_sd[i].linear_acceleration.y * 9.810 - aver_y)*(imu_sd[i].linear_acceleration.y * 9.810 - aver_y);

    aver_z += (imu_sd[i].linear_acceleration.z * 9.810 - aver_z) / (i + 1);
    ref_standing_z += (imu_sd[i].linear_acceleration.z * 9.810 - aver_z)*(imu_sd[i].linear_acceleration.z * 9.810 - aver_z);
  }
  if (imu_sd.size() > 0)
    var = sqrt(ref_standing_x+ref_standing_y+ref_standing_z) / imu_sd.size();
  return var;
}

double CalVar(std::deque<sensor_msgs::Imu> &imu_sd, double &imu_y_min, double &imu_y_max)
{
  double aver_x = 0.0, aver_y = 0.0, aver_z = 0.0;
  double ref_standing_x = 0.0, ref_standing_y = 0.0, ref_standing_z = 0.0;
  double var = 0.0;
  for (int i = 0; i < imu_sd.size(); i++) 
  {
    aver_x += (imu_sd[i].linear_acceleration.x - aver_x) / (i + 1);
    ref_standing_x += (imu_sd[i].linear_acceleration.x - aver_x)*(imu_sd[i].linear_acceleration.x - aver_x);

    aver_y += (imu_sd[i].linear_acceleration.y - aver_y) / (i + 1);
    ref_standing_y += (imu_sd[i].linear_acceleration.y - aver_y)*(imu_sd[i].linear_acceleration.y - aver_y);

    aver_z += (imu_sd[i].linear_acceleration.z - aver_z) / (i + 1);
    ref_standing_z += (imu_sd[i].linear_acceleration.z - aver_z)*(imu_sd[i].linear_acceleration.z - aver_z);

    if(imu_sd[i].linear_acceleration.y < imu_y_min)
        imu_y_min = imu_sd[i].linear_acceleration.y;
    if(imu_sd[i].linear_acceleration.y > imu_y_max)
        imu_y_max = imu_sd[i].linear_acceleration.y;
  }
  if(imu_sd.size() > 0)
    var = sqrt(ref_standing_x+ref_standing_y+ref_standing_z) / imu_sd.size();
  return var;
}

void CalRef_Fluct(sensor_msgs::Imu &imu, double &ref_standing, double &ref_swing)
{
  imu_sd.emplace_back(imu); 
  if(ref_standing == 0 || ref_swing == 0)
  {
    while (imu_sd.size() > imu_freq){
      imu_sd.pop_front();
    }
  }
  else
  {
    while (imu_sd.size() > gait_cycle/gait_cut/(joint_freq/imu_freq)){
      imu_sd.pop_front();
    }
  }

  //cal ref_standing
  if(ref_standing == 0 && imu_sd.size() >= 0.5*imu_freq)
  {
    ref_standing = CalVar(imu_sd);
    std::cout << "\033[32mref_standing = " << ref_standing << "\033[0m" << std::endl;
  }

  //cal ref_swing
  if(ref_swing == 0 && beg_swing != 0 && end_swing != 0)
  {
    while(imu_sd.front().header.stamp.toSec() > beg_swing + 0.01){
      imu_sd.pop_front();
    }
    if(imu_sd.size() == 0)
      std::cout << "error, all imu time < beg swing time, beg_swing not update!" << std::endl;
    while(imu_sd.back().header.stamp.toSec() < end_swing - 0.01){
      imu_sd.pop_back();
    }
    ref_swing = CalVar(imu_sd);
    imu_sd.clear();
    std::cout << "\033[32mref_swing = " << ref_swing << "\033[0m" << std::endl;
  }
}

bool Get_ContactEvent(double &ref_standing, double &ref_swing, ContactEvent &flg_contact, Eigen::Vector2d &dist_foot2plane)
{
  if(ref_standing == 0 || ref_swing == 0)     return false;
  if(flg_have_GaitCycle)
  {
		FootDist footdist_;
		footdist_.l_footdist = dist_foot2plane(0);
		footdist_.r_footdist = dist_foot2plane(1);
		footdist_sd.emplace_back(footdist_);
		while(footdist_sd.size() > gait_cycle/gait_cut){
			footdist_sd.pop_front();
		}
		
		if(footdist_.l_footdist > last_footl_dist*1.2)      flg_footl_stat = false;
		if(footdist_.r_footdist > last_footr_dist*1.2)      flg_footr_stat = false;
  }

  if(imu_sd.size() < gait_cycle/gait_cut/(joint_freq/imu_freq) || footdist_sd.size() < gait_cycle/gait_cut)    {return false;}

  contact_count++;
  flg_IMUchanged = false;
  double curr_var = CalVar(imu_sd);
  if(curr_var > ref_swing_multi*ref_swing)
  {
    if(flg_have_change)
    {
      flg_IMUchanged = true;
      last_footl_dist = dist_foot2plane(0);
      last_footr_dist = dist_foot2plane(1);
      int interval = gait_cycle/gait_cut / 5;
      // Quadratic polynomial curve fitting
      Eigen::VectorXd V_left(5), V_right(5);
      Eigen::MatrixXd T(5, 3);
      for (int i = 0; i < 5; i++)
      {
        int idx_footdist = i*interval;
        V_left(i) = (footdist_sd[idx_footdist-4].l_footdist + footdist_sd[idx_footdist-2].l_footdist + 
                    footdist_sd[idx_footdist].l_footdist + footdist_sd[idx_footdist+2].l_footdist + footdist_sd[idx_footdist+4].l_footdist) / 5.0;
        V_right(i)= (footdist_sd[idx_footdist-4].r_footdist + footdist_sd[idx_footdist-2].r_footdist + 
                    footdist_sd[idx_footdist].r_footdist + footdist_sd[idx_footdist+2].r_footdist + footdist_sd[idx_footdist+4].r_footdist) / 5.0;
        T(i, 0) = i*i;
        T(i, 1) = i;
        T(i, 2) = 1;
      }

      Eigen::VectorXd curve_left = T.householderQr().solve(V_left);
      Eigen::VectorXd curve_right= T.householderQr().solve(V_right);
      int rk_nums = 0, lk_nums = 0;
      Eigen::VectorXd tmpr(5);
      Eigen::VectorXd tmpl(5);
      for(int i = 0; i < 5; i++)
      {
        double l = 2*curve_left(0) * i + curve_left(1);
        double r = 2*curve_right(0)* i + curve_right(1);
        tmpr(i) = r;
        tmpl(i) = l;
        if(l < r) lk_nums++;
        else      rk_nums++;
      }

      if(rk_nums < lk_nums)
      {
        if(last_flg_contact == LeftFoot)
        {
          if(flg_footr_stat == true)  flg_contact = LeftFoot;
          else  flg_contact = NONE;
        }
        else  flg_contact = LeftFoot;    
      }
      else if(lk_nums < rk_nums)
      {
        if(last_flg_contact == RightFoot)
        {
          if(flg_footl_stat == true)  flg_contact = RightFoot;
          else  flg_contact = NONE;
        }
        else  flg_contact = RightFoot;
      }
      else  flg_contact = NONE;
      contact_count = 0;
    }
  }
  else if (curr_var <= 3.0*ref_standing)
  {
    flg_contact = BothFeet;
    contact_count = 0;
  }
  else if(contact_count > gait_cycle)
  {
    flg_contact = NONE;
  }

  contact_moment = false;
  if(flg_contact != NONE)
  {
    if(last_flg_contact != flg_contact)
    {
      contact_moment = true;
      _count = 10;
    }
  }
  if(_count > 0)
  {
    contact_moment = true;
    _count--;
  }
  last_flg_contact = flg_contact;

  if(flg_contact == LeftFoot)     flg_footl_stat = true;
  if(flg_contact == RightFoot)    flg_footr_stat = true;

  imu_var_sd.emplace_back(curr_var);
  if(imu_var_sd.size() > 20)      //[todo] e.g. param = 20.
    imu_var_sd.pop_front();

  for (double it : imu_var_sd)
  {
    flg_have_change = true;
    if(it >= ref_swing_multi*ref_swing){
      flg_have_change = false;
      break;
    }
  }
  
  sensor_msgs::Imu imu_var_msg; 
  imu_var_msg.header.stamp = ros::Time(time_current);
  imu_var_msg.header.frame_id = "camera_init";
  imu_var_msg.linear_acceleration.x = curr_var;
  pub_IMUVar.publish(imu_var_msg);
  return true;
}

void Set_FootState(const sensor_msgs::JointState &foot_state, V3D &ContactFootPos, V3D &ContactFootVel, ContactEvent &flg_contact)
{
	if(flg_contact == NONE)	return;
	if(flg_contact == LeftFoot)
	{
		ContactFootPos(0) = foot_state.position[0];
		ContactFootPos(1) = foot_state.position[1];
		ContactFootPos(2) = foot_state.position[2];

		ContactFootVel(0) = foot_state.velocity[0];
		ContactFootVel(1) = foot_state.velocity[1];
		ContactFootVel(2) = foot_state.velocity[2];
	}
	else if(flg_contact == RightFoot)
	{
		ContactFootPos(0) = foot_state.position[3];
		ContactFootPos(1) = foot_state.position[4];
		ContactFootPos(2) = foot_state.position[5];

		ContactFootVel(0) = foot_state.velocity[3];
		ContactFootVel(1) = foot_state.velocity[4];
		ContactFootVel(2) = foot_state.velocity[5];
	}
  else
  {
    ContactFootPos(0) = foot_state.position[0];
    ContactFootPos(1) = foot_state.position[1];
    ContactFootPos(2) = foot_state.position[2];

    ContactFootVel(0) = foot_state.velocity[0];
    ContactFootVel(1) = foot_state.velocity[1];
    ContactFootVel(2) = foot_state.velocity[2];
  }
}

void Cal_ContactHeight(V3D pc_pred, V3D &CHD_data)
{
  CHD_data.setZero();

  PointType foot_in_world_pred;
  foot_in_world_pred.x = pc_pred(0);
  foot_in_world_pred.y = pc_pred(1);
  foot_in_world_pred.z = pc_pred(2);
  PointVector search_result_, near_points_;
  std::vector<float> pointSearchDis_;
  ivox_->GetGroundClosestPoint(foot_in_world_pred, search_result_, pointSearchDis_, 0.3, 15);
  if(!need_hypPlane)
  {
    for (int j = 0; j < pointSearchDis_.size(); ++j)
    {
      if(pointSearchDis_[j] < 0.2);
        near_points_.push_back(search_result_[j]);
    }
    if(near_points_.size() < 10) return;
    double mean_point_z = 0.0;
    int N = 1;
    for(const auto& point : near_points_)
    {
      mean_point_z += (point.z - mean_point_z) / N;
      N++;
    }

    Eigen::Matrix<float, 4, 1> pca_result;
    float roughness = 0.0;
    float thre_ = 0.01;
    if(!esti_plane(pca_result, near_points_, thre_, near_points_.size(), roughness))  return;
    float r = pca_result(0) * foot_in_world_pred.x + pca_result(1) * foot_in_world_pred.y + pca_result(2) * foot_in_world_pred.z + pca_result(3);
    if(abs(foot_in_world_pred.z - mean_point_z) > 0.03 + solediff2model) return;

    CHD_data(0) = foot_in_world_pred.z - r*pca_result(2);
    CHD_data(1) = 1;
    CHD_data(2) = abs(roughness * pca_result(2));
  }
}

void MergeMeas(std::deque<sensor_msgs::Imu::Ptr> imu_deque, std::deque<sensor_msgs::JointState::Ptr> encoder_deque, std::deque<ImuJointData> &hybrid_deque)
{
  ImuJointData hybrid_data;
  int i = 0, j = 0;
  while (i < imu_deque.size() && j < encoder_deque.size() )
  {
    if(encoder_deque[j]->header.stamp.toSec() < imu_deque[i]->header.stamp.toSec())
    {
      hybrid_data.time_alignment = false;
      hybrid_data.joint = *encoder_deque[j];
      hybrid_data.data_type = 1;
      hybrid_data.timestamp = encoder_deque[j]->header.stamp.toSec();
      hybrid_deque.emplace_back(hybrid_data);
      j++;
    }
    else
    {
      if(encoder_deque[j]->header.stamp.toSec() == imu_deque[i]->header.stamp.toSec())
      {
        hybrid_data.time_alignment = true;
        hybrid_data.joint = *encoder_deque[j];
        j++;
      }
      else  hybrid_data.time_alignment = false;
      
      hybrid_data.imu = *imu_deque[i];
      hybrid_data.data_type = 0;
      hybrid_data.timestamp = imu_deque[i]->header.stamp.toSec();
      hybrid_deque.emplace_back(hybrid_data);
      i++;
    }
  }
  while (i < imu_deque.size())
  {
    hybrid_data.time_alignment = false;
    hybrid_data.imu = *imu_deque[i];
    hybrid_data.data_type = 0;
    hybrid_data.timestamp = imu_deque[i]->header.stamp.toSec();
    hybrid_deque.emplace_back(hybrid_data);
    i++;
  }
  while (j < encoder_deque.size())
  {
    hybrid_data.time_alignment = false;
    hybrid_data.joint = *encoder_deque[j];
    hybrid_data.data_type = 1;
    hybrid_data.timestamp = encoder_deque[j]->header.stamp.toSec();
    hybrid_deque.emplace_back(hybrid_data);
    j++;
  }

  imu_deque.clear();
  clear_imu = true;
  encoder_deque.clear();
  clear_encoder = true;

  // if(hybrid_deque.size() > 1000) 
  //   std::cout << "hybrid_deque size = " << hybrid_deque.size() << std::endl; // [todo]
}

void publish_init_map(const ros::Publisher & pubLaserCloudFullRes)
{
  sensor_msgs::PointCloud2 laserCloudmsg;    
  pcl::toROSMsg(*init_feats_world, laserCloudmsg);
  laserCloudmsg.header.stamp = ros::Time().fromSec(lidar_end_time);
  laserCloudmsg.header.frame_id = "camera_init";
  pubLaserCloudFullRes.publish(laserCloudmsg);
}

void publish_pc_frame_world(const ros::Publisher & pub_pc, Eigen::Vector3d pc)
{
  PointCloudXYZI::Ptr pc_in_world(new PointCloudXYZI(1, 1));
  pc_in_world->points[0].x = pc(0);
  pc_in_world->points[0].y = pc(1);
  pc_in_world->points[0].z = pc(2);
  sensor_msgs::PointCloud2 pc_msg;  
  pcl::toROSMsg(*pc_in_world, pc_msg);
  pc_msg.header.stamp = ros::Time().fromSec(lidar_end_time);
  pc_msg.header.frame_id = "camera_init";
  pub_pc.publish(pc_msg);
}

PointCloudXYZI::Ptr pcl_wait_save(new PointCloudXYZI());
void publish_frame_world(const ros::Publisher & pubLaserCloudFullRes, const ros::Publisher & pubPointDetaD)
{
  if (scan_pub_en)
  {
    PointCloudXYZI::Ptr laserCloudFullRes(feats_down_body);
    int size = laserCloudFullRes->points.size();
    PointCloudXYZI::Ptr laserCloudWorld(new PointCloudXYZI(size, 1));
      
    for (int i = 0; i < size; i++)
    {
      laserCloudWorld->points[i].x = feats_down_world->points[i].x;
      laserCloudWorld->points[i].y = feats_down_world->points[i].y;
      laserCloudWorld->points[i].z = feats_down_world->points[i].z;
      laserCloudWorld->points[i].intensity = feats_down_world->points[i].intensity; // feats_down_world->points[i].y;
    }
    sensor_msgs::PointCloud2 laserCloudmsg;
    pcl::toROSMsg(*laserCloudWorld, laserCloudmsg);
    
    laserCloudmsg.header.stamp = ros::Time().fromSec(lidar_end_time);
    laserCloudmsg.header.frame_id = "camera_init";
    pubLaserCloudFullRes.publish(laserCloudmsg);
  }
  
  /**************** save map ****************/
  /* 1. make sure you have enough memories
  /* 2. noted that pcd save will influence the real-time performences **/
  if (pcd_save_en)
  {
    int size = feats_down_world->points.size();
    PointCloudXYZI::Ptr   laserCloudWorld(new PointCloudXYZI(size, 1));

    for (int i = 0; i < size; i++)
    {
      laserCloudWorld->points[i].x = feats_down_world->points[i].x;
      laserCloudWorld->points[i].y = feats_down_world->points[i].y;
      laserCloudWorld->points[i].z = feats_down_world->points[i].z;
      laserCloudWorld->points[i].intensity = feats_down_world->points[i].intensity;
    }

    *pcl_wait_save += *laserCloudWorld;
    static int scan_wait_num = 0;
    scan_wait_num ++;
    if (pcl_wait_save->size() > 0 && pcd_save_interval > 0  && scan_wait_num >= pcd_save_interval)
    {
      pcd_index ++;
      string all_points_dir("/home/gjx/pcd.pcd");
      pcl::PCDWriter pcd_writer;
      cout << "current scan saved to /PCD/" << all_points_dir << endl;
      pcd_writer.writeBinary(all_points_dir, *pcl_wait_save);
      pcl_wait_save->clear();
      scan_wait_num = 0;
    }
  }
}

void publish_frame_body(const ros::Publisher & pubLaserCloudFull_body)
{
  int size = feats_undistort->points.size();
  PointCloudXYZI::Ptr laserCloudIMUBody(new PointCloudXYZI(size, 1));

  for (int i = 0; i < size; i++)
  {
    pointBodyLidarToIMU(&feats_undistort->points[i], &laserCloudIMUBody->points[i]);
  }

  sensor_msgs::PointCloud2 laserCloudmsg;
  pcl::toROSMsg(*laserCloudIMUBody, laserCloudmsg);
  laserCloudmsg.header.stamp = ros::Time().fromSec(lidar_end_time);
  laserCloudmsg.header.frame_id = "body";
  pubLaserCloudFull_body.publish(laserCloudmsg);
}

template<typename T>
void set_posestamp(T & out)
{
  out.position.x = kf.x_.pos(0);
  out.position.y = kf.x_.pos(1);
  out.position.z = kf.x_.pos(2);
  Eigen::Quaterniond q(kf.x_.rot);
  out.orientation.x = q.coeffs()[0];
  out.orientation.y = q.coeffs()[1];
  out.orientation.z = q.coeffs()[2];
  out.orientation.w = q.coeffs()[3];
}

void publish_odometry(const ros::Publisher & pubOdomAftMapped)
{
  odomAftMapped.header.frame_id = "camera_init";
  odomAftMapped.child_frame_id = "body";
  if (publish_odometry_without_downsample)
    odomAftMapped.header.stamp = ros::Time().fromSec(time_current);
  else
    odomAftMapped.header.stamp = ros::Time().fromSec(lidar_end_time);
  
  set_posestamp(odomAftMapped.pose.pose);
  pubOdomAftMapped.publish(odomAftMapped);

  static tf::TransformBroadcaster br;
  tf::Transform                   transform;
  tf::Quaternion                  q;
  transform.setOrigin(tf::Vector3(odomAftMapped.pose.pose.position.x, \
                                  odomAftMapped.pose.pose.position.y, \
                                  odomAftMapped.pose.pose.position.z));
  q.setW(odomAftMapped.pose.pose.orientation.w);
  q.setX(odomAftMapped.pose.pose.orientation.x);
  q.setY(odomAftMapped.pose.pose.orientation.y);
  q.setZ(odomAftMapped.pose.pose.orientation.z);
  transform.setRotation( q );
  br.sendTransform( tf::StampedTransform( transform, odomAftMapped.header.stamp, "camera_init", "body") );
}

void publish_path(const ros::Publisher pubPath)
{
  set_posestamp(msg_body_pose.pose);
  // msg_body_pose.header.stamp = ros::Time::now();
  msg_body_pose.header.stamp = ros::Time().fromSec(lidar_end_time);
  msg_body_pose.header.frame_id = "camera_init";
  static int jjj = 0;
  jjj++;
  // if (jjj % 2 == 0) // if path is too large, the rvis will crash
  {
    path.poses.emplace_back(msg_body_pose);
    pubPath.publish(path);
  }
}  

int main(int argc, char** argv)
{
  ros::init(argc, argv, "laserMapping");
  ros::NodeHandle nh("~");
  ros::AsyncSpinner spinner(0);
  spinner.start();
  readParameters(nh);
  cout<<"lidar_type: "<<lidar_type<<endl;
  ivox_ = std::make_shared<IVoxType>(ivox_options_);
  path.header.stamp    = ros::Time().fromSec(lidar_end_time);
  path.header.frame_id ="camera_init";

  /*** variables definition for counting ***/
  int frame_num = 0;
  double aver_time_consu = 0, aver_time_icp = 0, aver_time_match = 0, aver_time_incre = 0, aver_time_solve = 0, aver_time_propag = 0;

  memset(point_selected_surf, true, sizeof(point_selected_surf));
  downSizeFilterSurf.setLeafSize(filter_size_surf_min, filter_size_surf_min, filter_size_surf_min);
  downSizeFilterMap.setLeafSize(filter_size_map_min, filter_size_map_min, filter_size_map_min);
  
  Lidar_T_wrt_IMU<<VEC_FROM_ARRAY(extrinT);
  Lidar_R_wrt_IMU<<MAT_FROM_ARRAY(extrinR);
  Fixed_R_wrt_LiDAR<<MAT_FROM_ARRAY(extrinRfl);

  p_imu->lidar_type = p_pre->lidar_type = lidar_type;
  p_imu->imu_en = imu_en;

  kf.init_dyn_share_modified_3h(get_f, df_dx, h_model_output, h_model_IMU_output, h_model_Joint_output);
  Eigen::Matrix<double, 33, 33> P_init_output;
  Eigen::Matrix<double, 33, 32> P_init_output_;
  reset_cov(P_init_output);
  kf.change_P(P_init_output);
  Eigen::Matrix<double, 33, 33> Q_output = process_noise_cov();
  /*** debug record ***/
  open_file();

  /*** ROS subscribe initialization ***/
  ros::Subscriber sub_pcl = p_pre->lidar_type == AVIA ? \
      nh.subscribe(lid_topic, 200000, livox_pcl_cbk) : \
      nh.subscribe(lid_topic, 200000, standard_pcl_cbk);
  ros::Subscriber sub_imu = nh.subscribe(imu_topic, 200000, imu_cbk);
  ros::Subscriber sub_encoder = nh.subscribe(encoder_topic, 200000, encoder_cbk);

  ros::Publisher pubLaserCloudFullRes = nh.advertise<sensor_msgs::PointCloud2>("/cloud_registered", 1000);
  ros::Publisher pubPointDetaD = nh.advertise<sensor_msgs::PointCloud2>("/cloud_uncertain", 1000);
  ros::Publisher pubLaserCloudFullRes_body = nh.advertise<sensor_msgs::PointCloud2>("/cloud_registered_body", 1000);
  ros::Publisher pubLaserCloudMap = nh.advertise<sensor_msgs::PointCloud2>("/Laser_map", 1000);
  ros::Publisher pubOdomAftMapped = nh.advertise<nav_msgs::Odometry> ("/aft_mapped_to_init", 1000);
  ros::Publisher pubPath          = nh.advertise<nav_msgs::Path> ("/path", 1000);
  pub_footdist_hyp = nh.advertise<sensor_msgs::Imu>("/test/foot_dist2hyp", 1000, true);	// for vis test
  pub_IMUVar = nh.advertise<sensor_msgs::Imu>("/test/imu_var", 100, true);
  pub_Contact = nh.advertise<sensor_msgs::Imu>("/test/contact", 100, true);
  pub_pc = nh.advertise<sensor_msgs::PointCloud2>("/pc_in_world", 100, true);

  signal(SIGINT, SigHandle);
  ros::Rate loop_rate(500);
  bool status = ros::ok();
  while (status)
  {
    if (flg_exit) break;
    ros::spinOnce();
    if(sync_packages(Measures)) 
    {
      if (flg_reset)
      {
        ROS_WARN("reset when rosbag play back");
        p_imu->Reset();
        feats_undistort.reset(new PointCloudXYZI());

        kf.change_P(P_init_output);

        flg_first_scan = true;
        is_first_frame = true;
        flg_reset = false;
        init_map = false;
        ivox_.reset(new IVoxType(ivox_options_));
      }

      if (flg_first_scan)
      {
        algor_beg_time = omp_get_wtime();
        first_lidar_time = Measures.lidar_beg_time;
        flg_first_scan = false;
        if (first_imu_time < 1)
        {
          first_imu_time = imu_next.header.stamp.toSec();
          printf("first imu time: %f\n", first_imu_time);
        }
        time_current = 0.0;
        if(imu_en)
        {
          kf.x_.gravity << VEC_FROM_ARRAY(gravity);
          {
            while (Measures.lidar_beg_time > imu_next.header.stamp.toSec()) // if it is needed for the new map?
            {
              imu_deque.pop_front();
              if (imu_deque.empty())  {break;}
              imu_last = imu_next;
              imu_next = *(imu_deque.front());
            }
          }
        }
        else
        {
          kf.x_.gravity << VEC_FROM_ARRAY(gravity); //_init);
          kf.x_.acc << VEC_FROM_ARRAY(gravity); //_init);
          kf.x_.acc *= -1; 
          p_imu->imu_need_init_ = false;
        }     
        G_m_s2 = std::sqrt(gravity[0] * gravity[0] + gravity[1] * gravity[1] + gravity[2] * gravity[2]);
      }

      double t0,t1,t2,t3,t4,t5,match_start, solve_start;
      match_time = 0;
      solve_time = 0;
      propag_time = 0;
      update_time = 0;
      t0 = omp_get_wtime();

      /* IMU process */
      t1 = omp_get_wtime();
      p_imu->Process(Measures, feats_undistort);

      /*** downsample the feature points in a scan ***/
      if(space_down_sample)
      {
        downSizeFilterSurf.setInputCloud(feats_undistort);
        downSizeFilterSurf.filter(*feats_down_body);
        sort(feats_down_body->points.begin(), feats_down_body->points.end(), time_list); 
      }
      else
      {
        feats_down_body = Measures.lidar;
        sort(feats_down_body->points.begin(), feats_down_body->points.end(), time_list); 
      }
      time_seq = time_compressing<int>(feats_down_body);
      feats_down_size = feats_down_body->points.size();

      if (!p_imu->after_imu_init_)
      {
        if (!p_imu->imu_need_init_)
        {
          V3D tmp_gravity;
          // if (imu_en)
            tmp_gravity = - p_imu->mean_acc / p_imu->mean_acc.norm() * G_m_s2;

          M3D rot_init;
          p_imu->Set_init(tmp_gravity, rot_init);
          // rot_init = Eigen::Matrix3d::Identity();
          // kf.x_.gravity = tmp_gravity;
          kf.x_.rot = rot_init;
          kf.x_.acc = - rot_init.transpose() * kf.x_.gravity;
        }
        else
          continue;
      }
      /*** initialize the map ***/
      if(!init_map)
      {
        feats_down_world->resize(feats_undistort->size());
        for(int i = 0; i < feats_undistort->size(); i++)
        {
          pointBodyToWorld(&(feats_undistort->points[i]), &(feats_down_world->points[i]));
        }
        for (size_t i = 0; i < feats_down_world->size(); i++) 
        {
          init_feats_world->points.emplace_back(feats_down_world->points[i]);
        }
        if(init_feats_world->size() < init_map_size) 
        {init_map = false;}
        else
        {   
          ivox_->AddPoints(init_feats_world->points);
          publish_init_map(pubLaserCloudMap); //(pubLaserCloudFullRes);
          init_feats_world.reset(new PointCloudXYZI());
          init_map = true;
        }
        continue;
      }

      /*** ICP and Kalman filter update ***/
      normvec->resize(feats_down_size);
      feats_down_world->resize(feats_down_size);
      Nearest_Points.resize(feats_down_size);

      t2 = omp_get_wtime();
      /*** iterated state estimation ***/
      crossmat_list.reserve(feats_down_size);
      pbody_list.reserve(feats_down_size);

      for (size_t i = 0; i < feats_down_body->size(); i++)
      {   
        V3D point_this(feats_down_body->points[i].x, feats_down_body->points[i].y, feats_down_body->points[i].z);
        pbody_list[i] = point_this;
        point_this = Lidar_R_wrt_IMU * point_this + Lidar_T_wrt_IMU;
        M3D point_crossmat;
        point_crossmat << SKEW_SYM_MATRX(point_this);
        crossmat_list[i] = point_crossmat;
      }

      bool imu_upda_cov = false;
      effct_feat_num = 0;
      MergeMeas(imu_deque, encoder_deque, hybrid_deque);

      /**** point by point update ****/
      if (time_seq.size() > 0)
      {
        double pcl_beg_time = Measures.lidar_beg_time;
        idx = -1;
        for (k = 0; k < time_seq.size(); k++)
        {
          PointType &point_body  = feats_down_body->points[idx+time_seq[k]];
          time_current = point_body.curvature / 1000.0 + pcl_beg_time;        // ms/1000 + utc tims (s) = s + utc tims (s)
          if (is_first_frame)
          {
            if(imu_en)     
            {
              while (time_current > hybrid_data_next.timestamp && hybrid_data_next.data_type == 0)
              {
                hybrid_deque.pop_front();
                if(hybrid_deque.empty()) break;
                hybrid_data_last = hybrid_data_next;
                hybrid_data_next = hybrid_deque.front();
              }
              angvel_avr<<hybrid_data_last.imu.angular_velocity.x, hybrid_data_last.imu.angular_velocity.y, hybrid_data_last.imu.angular_velocity.z;
              acc_avr   <<hybrid_data_last.imu.linear_acceleration.x, hybrid_data_last.imu.linear_acceleration.y, hybrid_data_last.imu.linear_acceleration.z;
            }
            is_first_frame = false;
            imu_upda_cov = true;
            time_update_last = time_current;
            time_predict_last_const = time_current;
          }
          
          if(!hybrid_deque.empty())
          {
            bool last_hybrid_data = hybrid_data_next.timestamp == hybrid_deque.front().timestamp;
            while (hybrid_data_next.timestamp < time_predict_last_const && !hybrid_deque.empty())
            {
              if (!last_hybrid_data)
              {
                hybrid_data_last = hybrid_data_next;
                hybrid_data_next = hybrid_deque.front();
                break;
              }
              else
              {
                hybrid_deque.pop_front();
                if (hybrid_deque.empty()) break;
                hybrid_data_last = hybrid_data_next;
                hybrid_data_next = hybrid_deque.front();
              }
            }

            bool hybrid_data_comes = time_current > hybrid_data_next.timestamp;
            while (hybrid_data_comes)
            {
              if(hybrid_data_next.data_type == 0)
              {
                CalRef_Fluct(hybrid_data_next.imu, ref_standing, ref_swing);
                imu_upda_cov = true;
                angvel_avr<< hybrid_data_next.imu.angular_velocity.x, hybrid_data_next.imu.angular_velocity.y, hybrid_data_next.imu.angular_velocity.z;
                acc_avr   << hybrid_data_next.imu.linear_acceleration.x, hybrid_data_next.imu.linear_acceleration.y, hybrid_data_next.imu.linear_acceleration.z;
                /*** covariance update ***/
                double dt = hybrid_data_next.timestamp - time_predict_last_const;
                kf.predict(dt, Q_output, input_in, true, false);
                time_predict_last_const = hybrid_data_next.timestamp;
                {
                  double dt_cov = hybrid_data_next.timestamp - time_update_last; 
                  if (dt_cov > 0.0)
                  {
                    time_update_last = hybrid_data_next.timestamp;
                    double propag_imu_start = omp_get_wtime();
                    kf.predict(dt_cov, Q_output, input_in, false, true);
                    propag_time += omp_get_wtime() - propag_imu_start;
                    double solve_imu_start = omp_get_wtime();
                    kf.update_iterated_dyn_share_IMU();
                    solve_time += omp_get_wtime() - solve_imu_start;
                  }
                }
              }
              if(hybrid_data_next.data_type == 1 || hybrid_data_next.time_alignment == true)
              {
                sensor_msgs::JointState foot_state;
                foot_state = hybrid_data_next.joint;
                Eigen::Vector2d dist_foot2plane = CalDist_Foot2Plane(foot_state);
                if(dist_foot2plane != Eigen::Vector2d::Zero() && Get_GaitCycle(dist_foot2plane, foot_state.header.stamp.toSec(), gait_cycle))
                {
                  if(Get_ContactEvent(ref_standing, ref_swing, flg_contact, dist_foot2plane))
                  {
                    sensor_msgs::Imu contact_msg; 
                    if(hybrid_data_next.data_type == 1)
                    {
                      double dt_ = hybrid_data_next.timestamp - time_predict_last_const;
                      kf.predict(dt_, Q_output, input_in, true, false); 
                      time_predict_last_const = hybrid_data_next.timestamp;
                    }

                    if(flg_contact != prev_flg_contact)
                    {
                      if(contact_change_num == 0)
                      {
                        prev_phase_flg_contact = flg_contact;
                        contact_change_num++;
                      }
                      else
                      {
                        if(flg_contact == prev_phase_flg_contact) contact_change_num++;
                        else  contact_change_num = 0;
                      }
                      if(contact_change_num > 2 || flg_contact == BothFeet)
                      {
                        Set_FootState(foot_state, pos_contact_body, vel_contact_body, flg_contact);
                        kf.x_.pc = kf.x_.rot*pos_contact_body + kf.x_.pos;
                        Cal_ContactHeight(kf.x_.pc, CHD_data);
                        prev_flg_contact = flg_contact;
                      }
                    }
                    else if(flg_contact == BothFeet)
                    {
                      Set_FootState(foot_state, pos_contact_body, vel_contact_body, flg_contact);
                      kf.x_.pc = kf.x_.rot*pos_contact_body + kf.x_.pos;
                      Cal_ContactHeight(kf.x_.pc, CHD_data);
                      prev_flg_contact = flg_contact;
                    }
                    else
                      Set_FootState(foot_state, pos_contact_body, vel_contact_body, flg_contact);

                    if(flg_contact != NONE)
                      kf.update_iterated_dyn_share_Joint();

                    /* for vis */
                    if(flg_contact == LeftFoot)   contact_msg.linear_acceleration.x = 2;
                    if(flg_contact == RightFoot)  contact_msg.linear_acceleration.x = 1;
                    if(flg_contact == NONE)       contact_msg.linear_acceleration.x = 0;
                    if(flg_contact == BothFeet)   contact_msg.linear_acceleration.x = 3;
                    contact_msg.header.stamp = foot_state.header.stamp;
                    contact_msg.header.frame_id = "camera_init";
                    pub_Contact.publish(contact_msg);
                  }
                }
              }

              hybrid_deque.pop_front();
              if (hybrid_deque.empty()) break;
              hybrid_data_last = hybrid_data_next;
              hybrid_data_next = hybrid_deque.front();
              hybrid_data_comes = time_current > hybrid_data_next.timestamp;
            }
          }

          if (flg_reset)  {break;}

          double dt = time_current - time_predict_last_const;
          double propag_state_start = omp_get_wtime();
          kf.predict(dt, Q_output, input_in, true, false);
          propag_time += omp_get_wtime() - propag_state_start;
          time_predict_last_const = time_current;
          double t_update_start = omp_get_wtime();

          if (feats_down_size < 1)
          {
            ROS_WARN("No point, skip this scan!\n");
            idx += time_seq[k];
            continue;
          }
          if (!kf.update_iterated_dyn_share_modified())
          {
            idx = idx+time_seq[k];
            continue;
          }
          solve_start = omp_get_wtime();
          
          /******* Publish odometry *******/
          if (publish_odometry_without_downsample)
          {
            publish_odometry(pubOdomAftMapped);
          }

          for (int j = 0; j < time_seq[k]; j++)
          {
            PointType &point_body_j  = feats_down_body->points[idx+j+1];
            PointType &point_world_j = feats_down_world->points[idx+j+1];
            pointBodyToWorld(&point_body_j, &point_world_j);
          }
      
          solve_time += omp_get_wtime() - solve_start;
          update_time += omp_get_wtime() - t_update_start;
          idx += time_seq[k];
        }
      }
      
      /******* Publish odometry downsample *******/
      if (!publish_odometry_without_downsample)
        publish_odometry(pubOdomAftMapped);

      /*** add the feature points to map ***/
      t3 = omp_get_wtime();
      if(feats_down_size > 4)
      {
        MapIncremental();
      }
      t5 = omp_get_wtime();
      /******* Publish points *******/
      if (path_en)                         publish_path(pubPath);
      if (scan_pub_en || pcd_save_en)      publish_frame_world(pubLaserCloudFullRes, pubPointDetaD);
      if (scan_pub_en && scan_body_pub_en) publish_frame_body(pubLaserCloudFullRes_body);
      publish_pc_frame_world(pub_pc ,kf.x_.pc);
      
      /*** Debug variables Logging ***/
      if (runtime_pos_log)
      {
        frame_num ++;
        aver_time_consu = aver_time_consu * (frame_num - 1) / frame_num + (t5 - t0) / frame_num;
        {aver_time_icp = aver_time_icp * (frame_num - 1)/frame_num + update_time/frame_num;}
        aver_time_match = aver_time_match * (frame_num - 1)/frame_num + (match_time)/frame_num;
        aver_time_solve = aver_time_solve * (frame_num - 1)/frame_num + solve_time/frame_num;
        aver_time_propag = aver_time_propag * (frame_num - 1)/frame_num + propag_time / frame_num;
        T1[time_log_counter] = Measures.lidar_beg_time;
        s_plot[time_log_counter] = t5 - t0;
        s_plot2[time_log_counter] = feats_undistort->points.size();
        s_plot3[time_log_counter] = aver_time_consu;
        time_log_counter ++;
        printf("[ mapping ]: time: IMU + Map + Input Downsample: %0.6f ave match: %0.6f ave solve: %0.6f  ave ICP: %0.6f  map incre: %0.6f ave total: %0.6f icp: %0.6f propogate: %0.6f \n",t1-t0,aver_time_match,aver_time_solve,t3-t1,t5-t3,aver_time_consu*1000.0, aver_time_icp, aver_time_propag); 
        if (!publish_odometry_without_downsample)
        {
            euler_cur = SO3ToEuler(kf.x_.rot);
            fout_out << setw(20) << Measures.lidar_beg_time - first_lidar_time << " " << euler_cur.transpose() << " " << kf.x_.pos.transpose() << " " << kf.x_.vel.transpose() \
            <<" "<<kf.x_.omg.transpose()<<" "<<kf.x_.acc.transpose()<<" "<<kf.x_.gravity.transpose()<<" "<<kf.x_.bg.transpose()<<" "<<kf.x_.ba.transpose()<<" "<<feats_undistort->points.size()<<endl;
        }
      }
    }
    status = ros::ok();
    loop_rate.sleep();
  }
  //--------------------------save map-----------------------------------
  /* 1. make sure you have enough memories
  /* 2. noted that pcd save will influence the real-time performences **/
  if (pcl_wait_save->size() > 0 && pcd_save_en)
  {
    string file_name = string("scans.pcd");
    string all_points_dir(string("/home/gjx/") + file_name);
    pcl::PCDWriter pcd_writer;
    pcd_writer.writeBinary(all_points_dir, *pcl_wait_save);
  }
  fout_out.close();
  return 0;
}
