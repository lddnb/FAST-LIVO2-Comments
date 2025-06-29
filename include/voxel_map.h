/* 
This file is part of FAST-LIVO2: Fast, Direct LiDAR-Inertial-Visual Odometry.

Developer: Chunran Zheng <zhengcr@connect.hku.hk>

For commercial use, please contact me at <zhengcr@connect.hku.hk> or
Prof. Fu Zhang at <fuzhang@hku.hk>.

This file is subject to the terms and conditions outlined in the 'LICENSE' file,
which is included as part of this source code package.
*/

#ifndef VOXEL_MAP_H_
#define VOXEL_MAP_H_

#include "common_lib.h"
#include <Eigen/Dense>
#include <fstream>
#include <math.h>
#include <mutex>
#include <omp.h>
#include <pcl/common/io.h>
#include <ros/ros.h>
#include <thread>
#include <unistd.h>
#include <unordered_map>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#define VOXELMAP_HASH_P 116101
#define VOXELMAP_MAX_N 10000000000

static int voxel_plane_id = 0;

typedef struct VoxelMapConfig
{
  double max_voxel_size_;
  int max_layer_;
  int max_iterations_;
  std::vector<int> layer_init_num_;
  int max_points_num_;
  double planner_threshold_;
  double beam_err_;
  double dept_err_;
  double sigma_num_;
  bool is_pub_plane_map_;

  // config of local map sliding
  double sliding_thresh;
  bool map_sliding_en;
  int half_map_size;
} VoxelMapConfig;

typedef struct PointToPlane
{
  Eigen::Vector3d point_b_;
  Eigen::Vector3d point_w_;
  Eigen::Vector3d normal_;
  Eigen::Vector3d center_;
  Eigen::Matrix<double, 6, 6> plane_var_;
  M3D body_cov_;
  int layer_;
  double d_;
  double eigen_value_;
  bool is_valid_;
  float dis_to_plane_;
} PointToPlane;

typedef struct VoxelPlane
{
  Eigen::Vector3d center_;
  Eigen::Vector3d normal_;
  Eigen::Vector3d y_normal_;
  Eigen::Vector3d x_normal_;
  Eigen::Matrix3d covariance_;
  Eigen::Matrix<double, 6, 6> plane_var_;
  float radius_ = 0;
  float min_eigen_value_ = 1;
  float mid_eigen_value_ = 1;
  float max_eigen_value_ = 1;
  float d_ = 0;
  int points_size_ = 0;
  bool is_plane_ = false;
  bool is_init_ = false;
  int id_ = 0;
  bool is_update_ = false;
  VoxelPlane()
  {
    plane_var_ = Eigen::Matrix<double, 6, 6>::Zero();
    covariance_ = Eigen::Matrix3d::Zero();
    center_ = Eigen::Vector3d::Zero();
    normal_ = Eigen::Vector3d::Zero();
  }
} VoxelPlane;

class VOXEL_LOCATION
{
public:
  int64_t x, y, z;

  VOXEL_LOCATION(int64_t vx = 0, int64_t vy = 0, int64_t vz = 0) : x(vx), y(vy), z(vz) {}

  bool operator==(const VOXEL_LOCATION &other) const { return (x == other.x && y == other.y && z == other.z); }
};

// Hash value
namespace std
{
template <> struct hash<VOXEL_LOCATION>
{
  int64_t operator()(const VOXEL_LOCATION &s) const
  {
    using std::hash;
    using std::size_t;
    return ((((s.z) * VOXELMAP_HASH_P) % VOXELMAP_MAX_N + (s.y)) * VOXELMAP_HASH_P) % VOXELMAP_MAX_N + (s.x);
  }
};
} // namespace std

struct DS_POINT
{
  float xyz[3];
  float intensity;
  int count = 0;
};

/**
 * @brief 计算lidar点在lidar系下的协方差
 * 
 * @param pb 
 * @param range_inc 距离测量的标准差(1σ)，m
 * @param degree_inc 角度测量的标准差(1σ)，deg
 * @param cov
 */
void calcBodyCov(Eigen::Vector3d &pb, const float range_inc, const float degree_inc, Eigen::Matrix3d &cov);

class VoxelOctoTree
{

public:
  VoxelOctoTree() = default;
  std::vector<pointWithVar> temp_points_;
  VoxelPlane *plane_ptr_;
  int layer_;
  int octo_state_; // 0 is end of tree, 1 is not
  VoxelOctoTree *leaves_[8];
  double voxel_center_[3]; // x, y, z
  std::vector<int> layer_init_num_;
  float quater_length_;
  float planer_threshold_;
  int points_size_threshold_;
  int update_size_threshold_;
  int max_points_num_;
  int max_layer_;
  int new_points_;
  bool init_octo_;
  bool update_enable_;

  VoxelOctoTree(int max_layer, int layer, int points_size_threshold, int max_points_num, float planer_threshold)
      : max_layer_(max_layer), layer_(layer), points_size_threshold_(points_size_threshold), max_points_num_(max_points_num),
        planer_threshold_(planer_threshold)
  {
    temp_points_.clear();
    octo_state_ = 0;
    new_points_ = 0;
    update_size_threshold_ = 5;
    init_octo_ = false;
    update_enable_ = true;
    for (int i = 0; i < 8; i++)
    {
      leaves_[i] = nullptr;
    }
    plane_ptr_ = new VoxelPlane;
  }

  ~VoxelOctoTree()
  {
    for (int i = 0; i < 8; i++)
    {
      delete leaves_[i];
    }
    delete plane_ptr_;
  }
  /**
   * @brief 尝试拟合平面，计算特征值、特征向量和协方差
   * 
   * @param points 
   * @param plane 
   */
  void init_plane(const std::vector<pointWithVar> &points, VoxelPlane *plane);
  /**
   * @brief 当体素中点云数量达到一定阈值时，初始化这个体素
   * 
   */
  void init_octo_tree();
  /**
   * @brief 递归分割子节点
   * 
   */
  void cut_octo_tree();
  /**
   * @brief 递归往voxel的八叉树中插入点云
   * 
   * @param pv 
   */
  void UpdateOctoTree(const pointWithVar &pv);

  VoxelOctoTree *find_correspond(Eigen::Vector3d pw);
  VoxelOctoTree *Insert(const pointWithVar &pv);
};

/**
 * @brief 从ROS参数服务器加载体素地图配置
 * 
 * @param nh 
 * @param voxel_config 
 */
void loadVoxelConfig(ros::NodeHandle &nh, VoxelMapConfig &voxel_config);

class VoxelMapManager
{
public:
  VoxelMapManager() = default;
  VoxelMapConfig config_setting_;
  int current_frame_id_ = 0;
  ros::Publisher voxel_map_pub_;
  std::unordered_map<VOXEL_LOCATION, VoxelOctoTree *> voxel_map_;

  PointCloudXYZI::Ptr feats_undistort_;           // 点云去畸变后、降采样前，在Lidar系下的点云
  PointCloudXYZI::Ptr feats_down_body_;           // 点云降采样后，在Lidar系下的点云
  PointCloudXYZI::Ptr feats_down_world_;          // 点云降采样后，在World系下的点云

  M3D extR_;                                      // lidar到IMU的旋转外参 R_imu2lidar
  V3D extT_;                                      // lidar到IMU的平移外参 t_imu2lidar
  float build_residual_time, ekf_time;
  float ave_build_residual_time = 0.0;
  float ave_ekf_time = 0.0;
  int scan_count = 0;
  StatesGroup state_;
  V3D position_last_;

  V3D last_slide_position = {0,0,0};

  geometry_msgs::Quaternion geoQuat_;

  int feats_down_size_;
  int effct_feat_num_;
  std::vector<M3D> cross_mat_list_;
  std::vector<M3D> body_cov_list_;
  std::vector<pointWithVar> pv_list_;
  std::vector<PointToPlane> ptpl_list_;

  VoxelMapManager(VoxelMapConfig &config_setting, std::unordered_map<VOXEL_LOCATION, VoxelOctoTree *> &voxel_map)
      : config_setting_(config_setting), voxel_map_(voxel_map)
  {
    current_frame_id_ = 0;
    feats_undistort_.reset(new PointCloudXYZI());
    feats_down_body_.reset(new PointCloudXYZI());
    feats_down_world_.reset(new PointCloudXYZI());
  };

  /**
   * @brief IESKF的更新步骤
   * 
   * @param state_propagat IMU传播之后的状态量
   */
  void StateEstimation(StatesGroup &state_propagat);
  void TransformLidar(const Eigen::Matrix3d rot, const Eigen::Vector3d t, const PointCloudXYZI::Ptr &input_cloud,
                      pcl::PointCloud<pcl::PointXYZI>::Ptr &trans_cloud);

  /**
   * @brief 用第一帧点云初始化voxel地图
   * 
   */
  void BuildVoxelMap();
  V3F RGBFromVoxel(const V3D &input_point);

  /**
   * @brief 往voxel地图中插入新的点云，更新地图
   * 
   * @param input_points 
   */
  void UpdateVoxelMap(const std::vector<pointWithVar> &input_points);

  /**
   * @brief 构建点面残差约束，使用OMP并行化加速
   * 
   * @param pv_list 
   * @param ptpl_list 
   */
  void BuildResidualListOMP(std::vector<pointWithVar> &pv_list, std::vector<PointToPlane> &ptpl_list);

  /**
   * @brief 从根节点开始递归搜索一个体素，构建与当前点云的点面残差约束
   * 
   * @param pv 
   * @param current_octo 
   * @param current_layer 
   * @param is_sucess 
   * @param prob 
   * @param single_ptpl 
   */
  void build_single_residual(pointWithVar &pv, const VoxelOctoTree *current_octo, const int current_layer, bool &is_sucess, double &prob,
                             PointToPlane &single_ptpl);

  void pubVoxelMap();

  /**
   * @brief 根据行进的距离做地图滑窗，精简voxel地图，减少内存占用
   * 
   */
  void mapSliding();
  /**
   * @brief 清除地图中超出包围盒范围的voxel地图
   * 
   * @param x_max 
   * @param x_min 
   * @param y_max 
   * @param y_min 
   * @param z_max 
   * @param z_min 
   */
  void clearMemOutOfMap(const int& x_max,const int& x_min,const int& y_max,const int& y_min,const int& z_max,const int& z_min );

private:
  void GetUpdatePlane(const VoxelOctoTree *current_octo, const int pub_max_voxel_layer, std::vector<VoxelPlane> &plane_list);

  void pubSinglePlane(visualization_msgs::MarkerArray &plane_pub, const std::string plane_ns, const VoxelPlane &single_plane, const float alpha,
                      const Eigen::Vector3d rgb);
  void CalcVectQuation(const Eigen::Vector3d &x_vec, const Eigen::Vector3d &y_vec, const Eigen::Vector3d &z_vec, geometry_msgs::Quaternion &q);

  void mapJet(double v, double vmin, double vmax, uint8_t &r, uint8_t &g, uint8_t &b);
};
typedef std::shared_ptr<VoxelMapManager> VoxelMapManagerPtr;

#endif // VOXEL_MAP_H_