#ifndef COMMON_LIB_H
#define COMMON_LIB_H

#include <so3_math.h>
#include <Eigen/Eigen>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <eigen_conversions/eigen_msg.h>
#include <../include/IKFoM/IKFoM_toolkit/esekfom/esekfom.hpp>
#include <queue>
#include <sensor_msgs/JointState.h>

using namespace std;
using namespace Eigen;

typedef MTK::vect<3, double> vect3;
typedef MTK::SO3<double> SO3;
typedef MTK::S2<double, 98090, 10000, 1> S2; 
typedef MTK::vect<1, double> vect1;
typedef MTK::vect<2, double> vect2;

MTK_BUILD_MANIFOLD(state_ikfom,
((vect3, pos))                  //3
((SO3, rot))                    //6
((SO3, offset_R_L_I))           //9
((vect3, offset_T_L_I))         //12
((vect3, vel))                  //15
((vect3, omg))                  //18
((vect3, acc))                  //21
((vect3, gravity))              //24
((vect3, bg))                   //27
((vect3, ba))                   //30
((vect3, pc))                   //33
);

MTK_BUILD_MANIFOLD(input_ikfom,
((vect3, acc))
((vect3, gyro))
);

extern esekfom::esekf<state_ikfom, 33, input_ikfom> kf;
#define PBWIDTH 30
#define PBSTR "||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||"

#define PI_M (3.14159265358)
// #define G_m_s2 (9.81)    // Gravaty
#define INIT_COV   (0.0001)
#define NUM_MATCH_POINTS    (5)

#define VEC_FROM_ARRAY(v)        v[0],v[1],v[2]
#define VEC_FROM_ARRAY_SIX(v)    v[0],v[1],v[2],v[3],v[4],v[5]
#define MAT_FROM_ARRAY(v)        v[0],v[1],v[2],v[3],v[4],v[5],v[6],v[7],v[8]
#define CONSTRAIN(v,min,max)     ((v>min)?((v<max)?v:max):min)
#define ARRAY_FROM_EIGEN(mat)    mat.data(), mat.data() + mat.rows() * mat.cols()
#define STD_VEC_FROM_EIGEN(mat)  vector<decltype(mat)::Scalar> (mat.data(), mat.data() + mat.rows() * mat.cols())
#define DEBUG_FILE_DIR(name)     (string(string(ROOT_DIR) + "Log/"+ name))

typedef pcl::PointXYZINormal PointType;
typedef pcl::PointXYZRGB     PointTypeRGB;
typedef pcl::PointCloud<PointType>    PointCloudXYZI;
typedef pcl::PointCloud<PointTypeRGB> PointCloudXYZRGB;
typedef vector<PointType, Eigen::aligned_allocator<PointType>>  PointVector;
typedef Vector3d V3D;
typedef Matrix3d M3D;
typedef Vector3f V3F;
typedef Matrix3f M3F;

#define MD(a,b)  Matrix<double, (a), (b)>
#define VD(a)    Matrix<double, (a), 1>
#define MF(a,b)  Matrix<float, (a), (b)>
#define VF(a)    Matrix<float, (a), 1>

const M3D Eye3d(M3D::Identity());
const M3F Eye3f(M3F::Identity());
const V3D Zero3d(0, 0, 0);
const V3F Zero3f(0, 0, 0);

struct FootDist
{
  FootDist()
  {
    l_footdist = 0.0;
    l_footdist = 0.0;
    timestamp = 0.0;
  }
  double l_footdist;
  double r_footdist;
  double timestamp;
};

struct ImuJointData
{
  ImuJointData()
  {
    data_type = 0;
    time_alignment = false;
  }
  sensor_msgs::Imu imu;
  sensor_msgs::JointState joint;
  bool data_type;     // IMU = 0, Joint = 1
  double timestamp;
  bool time_alignment;
};

struct MeasureGroup     // Lidar data and imu dates for the curent process
{
  MeasureGroup()
  {
    lidar_beg_time = 0.0;
    lidar_last_time = 0.0;
    this->lidar.reset(new PointCloudXYZI());
  };
  double lidar_beg_time;
  double lidar_last_time;
  PointCloudXYZI::Ptr lidar;
  deque<sensor_msgs::Imu::ConstPtr> imu;
  deque<sensor_msgs::JointState::ConstPtr> joint;
};

template <typename T>
T calc_dist(PointType p1, PointType p2){
  T d = (p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y) + (p1.z - p2.z) * (p1.z - p2.z);
  return d;
}

template <typename T>
T calc_dist(Eigen::Vector3d p1, PointType p2){
  T d = (p1(0) - p2.x) * (p1(0) - p2.x) + (p1(1) - p2.y) * (p1(1) - p2.y) + (p1(2) - p2.z) * (p1(2) - p2.z);
  return d;
}

template<typename T>
std::vector<int> time_compressing(const PointCloudXYZI::Ptr &point_cloud)
{
  int points_size = point_cloud->points.size();
  int j = 0;
  std::vector<int> time_seq;
  time_seq.reserve(points_size);
  for(int i = 0; i < points_size - 1; i++)
  {
    j++;
    if (point_cloud->points[i+1].curvature > point_cloud->points[i].curvature)
    {
      time_seq.emplace_back(j);
      j = 0;
    }
  }
  time_seq.emplace_back(j+1);
  return time_seq;
}

/* comment
plane equation: Ax + By + Cz + D = 0
convert to: A/D*x + B/D*y + C/D*z = -1
solve: A0*x0 = b0
where A0_i = [x_i, y_i, z_i], x0 = [A/D, B/D, C/D]^T, b0 = [-1, ..., -1]^T
normvec:  normalized x0
*/
template<typename T>
bool esti_normvector(Matrix<T, 3, 1> &normvec, const PointVector &point, const T &threshold, const int &point_num)
{
  MatrixXf A(point_num, 3);
  MatrixXf b(point_num, 1);
  b.setOnes();
  b *= -1.0f;

  for (int j = 0; j < point_num; j++)
  {
    A(j,0) = point[j].x;
    A(j,1) = point[j].y;
    A(j,2) = point[j].z;
  }
  normvec = A.colPivHouseholderQr().solve(b);
  
  for (int j = 0; j < point_num; j++)
  {
    if (fabs(normvec(0) * point[j].x + normvec(1) * point[j].y + normvec(2) * point[j].z + 1.0f) > threshold)
    {
      return false;
    }
  }
  normvec.normalize();
  return true;
}

template<typename T>
bool esti_plane(Matrix<T, 4, 1> &pca_result, const PointVector &point, const T &threshold, bool &en_gnd_point,  bool &en_change_gnd)
{
  Matrix<T, NUM_MATCH_POINTS, 3> A;
  Matrix<T, NUM_MATCH_POINTS, 1> b;
  A.setZero();
  b.setOnes();
  b *= -1.0f;

  for (int j = 0; j < NUM_MATCH_POINTS; j++)
  {
    A(j,0) = point[j].x;
    A(j,1) = point[j].y;
    A(j,2) = point[j].z;
  }
  
  Matrix<T, 3, 1> normvec = A.colPivHouseholderQr().solve(b);
  T n = normvec.norm();
  pca_result(0) = normvec(0) / n;
  pca_result(1) = normvec(1) / n;
  pca_result(2) = normvec(2) / n;
  pca_result(3) = 1.0 / n;

  for (int j = 0; j < NUM_MATCH_POINTS; j++)
  {
    if (fabs(pca_result(0) * point[j].x + pca_result(1) * point[j].y + pca_result(2) * point[j].z + pca_result(3)) > threshold)
    {
      return false;
    }
  }
  return true;
}

template<typename T>
bool esti_plane(Matrix<T, 4, 1> &pca_result, const PointVector &point, const T &threshold, int point_size)
{
  Eigen::Matrix<T, Eigen::Dynamic, 3> A(point_size, 3);
  Eigen::Matrix<T, Eigen::Dynamic, 1> b(point_size);
  A.setZero();
  b.setOnes();
  b *= -1.0f;

  for (int j = 0; j < point_size; j++)
  {
    A(j,0) = point[j].x;
    A(j,1) = point[j].y;
    A(j,2) = point[j].z;
  }

  Matrix<T, 3, 1> normvec = A.colPivHouseholderQr().solve(b);
  T n = normvec.norm();
  pca_result(0) = normvec(0) / n;
  pca_result(1) = normvec(1) / n;
  pca_result(2) = normvec(2) / n;
  pca_result(3) = 1.0 / n;

  for (int j = 0; j < point_size; j++)
  {
    if (fabs(pca_result(0) * point[j].x + pca_result(1) * point[j].y + pca_result(2) * point[j].z + pca_result(3)) > threshold)
    {
      return false;
    }
  }
  return true;
}

template<typename T>
bool esti_plane_real(Matrix<T, 4, 1> &pca_result, const PointVector &point, const T &threshold, int point_size, int max_iter)
{
  PointVector point_revise = point;
  for(int it = 0; it < max_iter; it++)
  {
    int point_size_revise = point_revise.size();
    if(point_size_revise < 5) {std::cout << "points fit less than 5... " << std::endl; break;}
    Eigen::Matrix<T, Eigen::Dynamic, 3> A(point_size_revise, 3);
    Eigen::Matrix<T, Eigen::Dynamic, 1> b(point_size_revise);
    A.setZero();
    b.setOnes();
    b *= -1.0f;

    for (int j = 0; j < point_size_revise; j++)
    {
      A(j,0) = point_revise[j].x;
      A(j,1) = point_revise[j].y;
      A(j,2) = point_revise[j].z;
    }
    Matrix<T, 3, 1> normvec = A.colPivHouseholderQr().solve(b);
    T n = normvec.norm();
    pca_result(0) = normvec(0) / n;
    pca_result(1) = normvec(1) / n;
    pca_result(2) = normvec(2) / n;
    pca_result(3) = 1.0 / n;

    bool converge = true;
    for (int j = 0; j < point_size_revise; j++)
    {
      if (fabs(pca_result(0) * point_revise[j].x + pca_result(1) * point_revise[j].y + pca_result(2) * point_revise[j].z + pca_result(3)) > threshold)
      {
        if(point_revise.size() <= 5)
            break;
        point_revise.erase(point_revise.begin() + j);
        j--;
        point_size_revise--;
        converge = false;
        // return false;
      }
    }

    if(converge)  break;
  }
  return true;
}

template<typename T>
bool esti_plane(Matrix<T, 4, 1> &pca_result, const PointVector &point, const T &threshold, int point_size, float &roughness)
{
  Eigen::Matrix<T, Eigen::Dynamic, 3> A(point_size, 3);
  Eigen::Matrix<T, Eigen::Dynamic, 1> b(point_size);
  A.setZero();
  b.setOnes();
  b *= -1.0f;

  for (int j = 0; j < point_size; j++)
  {
    A(j,0) = point[j].x;
    A(j,1) = point[j].y;
    A(j,2) = point[j].z;
  }

  Matrix<T, 3, 1> normvec = A.colPivHouseholderQr().solve(b);
  T n = normvec.norm();
  pca_result(0) = normvec(0) / n;
  pca_result(1) = normvec(1) / n;
  pca_result(2) = normvec(2) / n;
  pca_result(3) = 1.0 / n;
  
  for (int j = 0; j < point_size; j++)
  {
    roughness += pca_result(0)*point[j].x + pca_result(1)*point[j].y + pca_result(2)*point[j].z + pca_result(3);
  }
  roughness = roughness/point_size;
  if(roughness > threshold) return false;
  return true;
}

#endif