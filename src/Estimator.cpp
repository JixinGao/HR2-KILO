#include "Estimator.h"

PointCloudXYZI::Ptr normvec(new PointCloudXYZI(100000, 1));
std::vector<int> time_seq;
PointCloudXYZI::Ptr feats_down_body(new PointCloudXYZI(10000, 1));
PointCloudXYZI::Ptr feats_down_world(new PointCloudXYZI(10000, 1));
std::vector<V3D> pbody_list;
std::vector<PointVector> Nearest_Points; 
std::shared_ptr<IVoxType> ivox_ = nullptr;                    // localmap in ivox
std::vector<float> pointSearchSqDis(NUM_MATCH_POINTS);
bool   point_selected_surf[100000] = {0};
std::vector<M3D> crossmat_list;
int effct_feat_num = 0;
int k = 0;
int idx = -1;
esekfom::esekf<state_ikfom, 33, input_ikfom> kf;
input_ikfom input_in;
V3D angvel_avr, acc_avr, acc_avr_norm;
int feats_down_size = 0;  
V3D Lidar_T_wrt_IMU(Zero3d);
M3D Lidar_R_wrt_IMU(Eye3d);
M3D Fixed_R_wrt_LiDAR(Eye3d);
double G_m_s2 = 9.81;

V3D pos_contact_body(Zero3d);
V3D vel_contact_body(Zero3d);
bool contact_moment = false;
V3D CHD_data(Zero3d);

Eigen::Matrix<double, 33, 33> process_noise_cov()
{
	Eigen::Matrix<double, 33, 33> cov;
	cov.setZero();
	cov.block<3, 3>(12, 12).diagonal() << vel_cov, vel_cov, vel_cov;
	cov.block<3, 3>(15, 15).diagonal() << gyr_cov_output, gyr_cov_output, gyr_cov_output;
	cov.block<3, 3>(18, 18).diagonal() << acc_cov_output, acc_cov_output, acc_cov_output;
	cov.block<3, 3>(24, 24).diagonal() << b_gyr_cov, b_gyr_cov, b_gyr_cov;
	cov.block<3, 3>(27, 27).diagonal() << b_acc_cov, b_acc_cov, b_acc_cov;
	cov.block<3, 3>(30, 30).diagonal() << b_pc_cov, b_pc_cov, b_pc_cov;
	return cov;
}

Eigen::Matrix<double, 33, 1> get_f(state_ikfom &s, const input_ikfom &in)
{
	Eigen::Matrix<double, 33, 1> res = Eigen::Matrix<double, 33, 1>::Zero();			
	vect3 a_inertial = s.rot * s.acc; // .normalized()
	for(int i = 0; i < 3; i++ ){
		res(i) = s.vel[i];
		res(i + 3) = s.omg[i];
		res(i + 12) = a_inertial[i] + s.gravity[i];
	}
	return res;
}

Eigen::Matrix<double, 33, 33> df_dx(state_ikfom &s, const input_ikfom &in)
{
	Eigen::Matrix<double, 33, 33> cov = Eigen::Matrix<double, 33, 33>::Zero(); 
	cov.template block<3, 3>(0, 12) = Eigen::Matrix3d::Identity();
	cov.template block<3, 3>(12, 3) = -s.rot*MTK::hat(s.acc); // .normalized().toRotationMatrix()
	cov.template block<3, 3>(12, 18) = s.rot; //.normalized().toRotationMatrix();
	cov.template block<3, 3>(12, 21) = Eigen::Matrix3d::Identity(); // grav_matrix; 
	cov.template block<3, 3>(3, 15) = Eigen::Matrix3d::Identity(); 
	cov.template block<3, 3>(30, 3) = -s.rot;
	return cov;
}

void h_model_output(state_ikfom &s, Eigen::Matrix3d cov_p, Eigen::Matrix3d cov_R, esekfom::dyn_share_modified<double> &ekfom_data)
{
	bool match_in_map = false;
	VF(4) pabcd;
	pabcd.setZero();
	normvec->resize(time_seq[k]);
	
	int effect_num_k = 0;
	for (int j = 0; j < time_seq[k]; j++)
	{
		PointType &point_body_j  = feats_down_body->points[idx+j+1]; 
		PointType &point_world_j = feats_down_world->points[idx+j+1];

		pointBodyToWorld(&point_body_j, &point_world_j);
		V3D p_body = pbody_list[idx+j+1];
		double p_norm = p_body.norm();
		V3D p_world;
		p_world << point_world_j.x, point_world_j.y, point_world_j.z;

    auto &points_near = Nearest_Points[idx+j+1];
    ivox_->GetClosestPoint(point_world_j, points_near, NUM_MATCH_POINTS);
    bool en_gnd_point = false;
    bool en_change_gnd = false;
    if ((points_near.size() < NUM_MATCH_POINTS))
    {
      point_selected_surf[idx+j+1] = false;
    }
    else
    {
      point_selected_surf[idx+j+1] = false;
      if (esti_plane(pabcd, points_near, plane_thr, en_gnd_point, en_change_gnd))
      {
        float pd2 = fabs(pabcd(0) * point_world_j.x + pabcd(1) * point_world_j.y + pabcd(2) * point_world_j.z + pabcd(3));
        if (p_norm > match_s * pd2 * pd2)
        {
          point_selected_surf[idx+j+1] = true;
          normvec->points[j].x = pabcd(0);
          normvec->points[j].y = pabcd(1);
          normvec->points[j].z = pabcd(2);
          normvec->points[j].intensity = pabcd(3);
          effect_num_k ++;
        }
      }  
    }
	}
	if (effect_num_k == 0) 
	{
		ekfom_data.valid = false;
		return;
	}
	ekfom_data.M_Noise = laser_point_cov;
	ekfom_data.h_x.resize(effect_num_k, 12);
	ekfom_data.h_x = Eigen::MatrixXd::Zero(effect_num_k, 12);
	ekfom_data.z.resize(effect_num_k);
	int m = 0;
	for (int j = 0; j < time_seq[k]; j++)
	{
		if(point_selected_surf[idx+j+1])
		{
			V3D norm_vec(normvec->points[j].x, normvec->points[j].y, normvec->points[j].z);
      M3D point_crossmat = crossmat_list[idx+j+1];
      V3D C(s.rot.transpose() * norm_vec); // conjugate().normalized()
      V3D A(point_crossmat * C);
      ekfom_data.h_x.block<1, 12>(m, 0) << norm_vec(0), norm_vec(1), norm_vec(2), VEC_FROM_ARRAY(A), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
			ekfom_data.z(m) = -norm_vec(0) * feats_down_world->points[idx+j+1].x -norm_vec(1) * feats_down_world->points[idx+j+1].y -norm_vec(2) * feats_down_world->points[idx+j+1].z-normvec->points[j].intensity;
			m++;
		}
	}
	effct_feat_num += effect_num_k;
}

void h_model_IMU_output(state_ikfom &s, esekfom::dyn_share_modified<double> &ekfom_data)
{
  std::memset(ekfom_data.satu_check, false, 6);
	ekfom_data.z_IMU.block<3,1>(0, 0) = angvel_avr - s.omg - s.bg;
	ekfom_data.z_IMU.block<3,1>(3, 0) = acc_avr * G_m_s2 / acc_norm - s.acc - s.ba;
	double imu_meas_omg_cov_ad = 0.1;
	double imu_meas_acc_cov_ad = 0.1;
	if(contact_moment)
	{
		imu_meas_omg_cov_ad = 2.0*imu_meas_omg_cov;
		imu_meas_acc_cov_ad = 2.0*imu_meas_acc_cov;
	}
	else
	{
		imu_meas_omg_cov_ad = imu_meas_omg_cov;
		imu_meas_acc_cov_ad = imu_meas_acc_cov;
	}
  ekfom_data.R_IMU << imu_meas_omg_cov_ad, imu_meas_omg_cov_ad, imu_meas_omg_cov_ad, imu_meas_acc_cov_ad, imu_meas_acc_cov_ad, imu_meas_acc_cov_ad;	// 6 dim cov
	if(check_satu)
	{
		if(fabs(angvel_avr(0)) >= 0.99 * satu_gyro)
		{
			ekfom_data.satu_check[0] = true; 
			ekfom_data.z_IMU(0) = 0.0;
		}
		
		if(fabs(angvel_avr(1)) >= 0.99 * satu_gyro) 
		{
			ekfom_data.satu_check[1] = true;
			ekfom_data.z_IMU(1) = 0.0;
		}
		
		if(fabs(angvel_avr(2)) >= 0.99 * satu_gyro)
		{
			ekfom_data.satu_check[2] = true;
			ekfom_data.z_IMU(2) = 0.0;
		}
		
		if(fabs(acc_avr(0)) >= 0.99 * satu_acc)
		{
			ekfom_data.satu_check[3] = true;
			ekfom_data.z_IMU(3) = 0.0;
		}

		if(fabs(acc_avr(1)) >= 0.99 * satu_acc) 
		{
			ekfom_data.satu_check[4] = true;
			ekfom_data.z_IMU(4) = 0.0;
		}

		if(fabs(acc_avr(2)) >= 0.99 * satu_acc) 
		{
			ekfom_data.satu_check[5] = true;
			ekfom_data.z_IMU(5) = 0.0;
		}
	}
}

void h_model_Joint_output(state_ikfom &s, esekfom::dyn_share_modified<double> &ekfom_data)
{
	ekfom_data.h_x_Joint.resize(7, 33);
	ekfom_data.h_x_Joint = Eigen::MatrixXd::Zero(7, 33);

	M3D angvel_avr_cross, A_cross, pos_cnt_b_cross;
	angvel_avr_cross << SKEW_SYM_MATRX(s.omg);
	V3D tmp = angvel_avr_cross * pos_contact_body + vel_contact_body;
	A_cross << SKEW_SYM_MATRX(tmp);

	V3D pc_w = s.rot * pos_contact_body + s.pos;		//for height z
	Eigen::Matrix<double, 1, 1> tmp_tmp;
	tmp_tmp(0,0) = CHD_data(0) - pc_w(2);

	pos_cnt_b_cross << SKEW_SYM_MATRX(pos_contact_body);

	ekfom_data.h_x_Joint.block<3, 3>(0, 3) = -s.rot * A_cross;
	ekfom_data.h_x_Joint.block<3, 3>(0, 12)= Eigen::Matrix3d::Identity();
	ekfom_data.h_x_Joint.block<3, 3>(0, 15)= -s.rot * pos_cnt_b_cross;

	ekfom_data.h_x_Joint.block<3, 3>(3, 0) = -Eigen::Matrix3d::Identity();
	ekfom_data.h_x_Joint.block<3, 3>(3, 3) = s.rot * pos_cnt_b_cross;
	ekfom_data.h_x_Joint.block<3, 3>(3, 30) = Eigen::Matrix3d::Identity();

	ekfom_data.h_x_Joint.block<1, 1>(6, 32) = Eigen::MatrixXd::Identity(1,1);

	ekfom_data.z_Joint.block<3, 1>(0, 0) = s.vel + s.rot * (angvel_avr_cross * pos_contact_body + vel_contact_body);
	ekfom_data.z_Joint.block<3, 1>(3, 0) = s.pc - s.rot * pos_contact_body - s.pos;
	ekfom_data.z_Joint.block<1, 1>(6, 0) = tmp_tmp;
	ekfom_data.R_Joint << joint_meas_vel_cov, joint_meas_vel_cov, joint_meas_vel_cov, joint_meas_pos_cov, joint_meas_pos_cov, joint_meas_pos_cov, 0.1;
	ekfom_data.height_check = true;
	if(CHD_data(1) == 0)
	{
		ekfom_data.height_check = false;
		ekfom_data.h_x_Joint.block<1, 1>(6, 32) = Eigen::MatrixXd::Zero(1,1);
		ekfom_data.z_Joint.block<1, 1>(6, 0) = Eigen::MatrixXd::Zero(1,1);
	}
}

void pointBodyToWorld(PointType const * const pi, PointType * const po)
{    
  V3D p_body(pi->x, pi->y, pi->z);
  V3D p_global;
  p_global = kf.x_.rot * (Lidar_R_wrt_IMU * p_body + Lidar_T_wrt_IMU) + kf.x_.pos;	//IMU->LIDAR->World

  po->x = p_global(0);
  po->y = p_global(1);
  po->z = p_global(2);
  po->intensity = pi->intensity;
	po->normal_x = pi->normal_x;
}

void footIMUtoWorld(PointType const * const pi, PointType * const po)
{
	V3D p_body(pi->x, pi->y, pi->z);
	V3D p_global;
	p_global = kf.x_.rot * p_body + kf.x_.pos;		//IMU->World

	po->x = p_global(0);
  po->y = p_global(1);
  po->z = p_global(2);
  po->intensity = pi->intensity;
	po->normal_x = pi->normal_x;
}

void setfoot(const sensor_msgs::JointState &foot_state, PointType &footL, PointType &footR)
{
	footL.x = foot_state.position[0];
	footL.y = foot_state.position[1];
	footL.z = foot_state.position[2];
	footR.x = foot_state.position[3];
	footR.y = foot_state.position[4];
	footR.z = foot_state.position[5];
}
