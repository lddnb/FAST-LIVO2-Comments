/* 
This file is part of FAST-LIVO2: Fast, Direct LiDAR-Inertial-Visual Odometry.

Developer: Chunran Zheng <zhengcr@connect.hku.hk>

For commercial use, please contact me at <zhengcr@connect.hku.hk> or
Prof. Fu Zhang at <fuzhang@hku.hk>.

This file is subject to the terms and conditions outlined in the 'LICENSE' file,
which is included as part of this source code package.
*/

#ifndef IMU_PROCESSING_H
#define IMU_PROCESSING_H

#include <Eigen/Eigen>
#include "common_lib.h"
#include <condition_variable>
#include <nav_msgs/Odometry.h>
#include <utils/so3_math.h>
#include <fstream>
const bool time_list(PointType &x, PointType &y) { return (x.curvature < y.curvature); }

/// *************IMU Process and undistortion
class ImuProcess
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ImuProcess();
  ~ImuProcess();

  /**
   * @brief 重置类对象
   * 
   */
  void Reset();
  void Reset(double start_timestamp, const sensor_msgs::ImuConstPtr &lastimu);
  /**
   * @brief 设置 T_imu2lidar 外参
   * 
   * @param transl 
   * @param rot 
   */
  void set_extrinsic(const V3D &transl, const M3D &rot);
  void set_extrinsic(const V3D &transl);
  void set_extrinsic(const MD(4, 4) & T);
  /****** 设置各种协方差 ******/
  void set_gyr_cov_scale(const V3D &scaler);
  void set_acc_cov_scale(const V3D &scaler);
  void set_gyr_bias_cov(const V3D &b_g);
  void set_acc_bias_cov(const V3D &b_a);
  void set_inv_expo_cov(const double &inv_expo);
  /**************************/

  /**
   * @brief 设置IMU初始化所需的帧数
   * 
   * @param num 
   */
  void set_imu_init_frame_num(const int &num);
  /**
   * @brief 设置不使用IMU
   * 
   */
  void disable_imu();
  /**
   * @brief 关闭对重力的估计
   * 
   */
  void disable_gravity_est();
  /**
   * @brief 关闭对bg和ba的估计
   * 
   */
  void disable_bias_est();
  /**
   * @brief 关闭对曝光时间的估计
   * 
   */
  void disable_exposure_est();
  /**
   * @brief 处理IMU数据的主函数
   * 
   * @param lidar_meas 
   * @param stat 
   * @param cur_pcl_un_ 
   */
  void Process2(LidarMeasureGroup &lidar_meas, StatesGroup &stat, PointCloudXYZI::Ptr cur_pcl_un_);
  /**
   * @brief 完成IMU状态传播和点云去畸变
   * 
   * @param lidar_meas 
   * @param state_inout 
   * @param pcl_out 
   */
  void UndistortPcl(LidarMeasureGroup &lidar_meas, StatesGroup &state_inout, PointCloudXYZI &pcl_out);

  ofstream fout_imu;
  double IMU_mean_acc_norm;
  V3D unbiased_gyr;

  V3D cov_acc;                                    // 加速度的协方差
  V3D cov_gyr;                                    // 角速度的协方差 
  V3D cov_bias_gyr;                               // 角速度的bias的协方差
  V3D cov_bias_acc;                               // 加速度的bias的协方差
  double cov_inv_expo;                            // 曝光时间倒数的协方差
  double first_lidar_time;                        // 记录第一帧的激光时间戳，用于计算每一帧IMU的时间偏移，用于日志输出
  bool imu_time_init = false;                     // tau是否已经完成初始化
  bool imu_need_init = true;                      // 是否需要进行IMU初始化
  M3D Eye3d;                                      // 单位矩阵
  V3D Zero3d;                                     // 零向量
  int lidar_type;                                 // 雷达类型

private:
  void IMU_init(const MeasureGroup &meas, StatesGroup &state, int &N);
  /**
   * @brief 无IMU数据时，仅根据当前状态向前传播，以及对点云去畸变
   * 
   * @param meas 
   * @param state_inout 
   * @param pcl_out 
   */
  void Forward_without_imu(LidarMeasureGroup &meas, StatesGroup &state_inout, PointCloudXYZI &pcl_out);
  PointCloudXYZI pcl_wait_proc;
  sensor_msgs::ImuConstPtr last_imu;
  PointCloudXYZI::Ptr cur_pcl_un_;
  vector<Pose6D> IMUpose;
  M3D Lid_rot_to_IMU;
  V3D Lid_offset_to_IMU;
  V3D mean_acc;
  V3D mean_gyr;
  V3D angvel_last;
  V3D acc_s_last;
  double last_prop_end_time;
  double time_last_scan;
  int init_iter_num = 1, MAX_INI_COUNT = 20;
  bool b_first_frame = true;
  bool imu_en = true;
  bool gravity_est_en = true;
  bool ba_bg_est_en = true;
  bool exposure_estimate_en = true;
};
typedef std::shared_ptr<ImuProcess> ImuProcessPtr;
#endif