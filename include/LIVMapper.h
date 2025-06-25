/* 
This file is part of FAST-LIVO2: Fast, Direct LiDAR-Inertial-Visual Odometry.

Developer: Chunran Zheng <zhengcr@connect.hku.hk>

For commercial use, please contact me at <zhengcr@connect.hku.hk> or
Prof. Fu Zhang at <fuzhang@hku.hk>.

This file is subject to the terms and conditions outlined in the 'LICENSE' file,
which is included as part of this source code package.
*/

#ifndef LIV_MAPPER_H
#define LIV_MAPPER_H

#include "IMU_Processing.h"
#include "vio.h"
#include "preprocess.h"
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <nav_msgs/Path.h>
#include <vikit/camera_loader.h>

class LIVMapper
{
public:
  LIVMapper(ros::NodeHandle &nh);
  ~LIVMapper();
  /**
 * @brief 创建ROS的订阅者和发布者
 * 
 * @param nh 
 * @param it 
 */
  void initializeSubscribersAndPublishers(ros::NodeHandle &nh, image_transport::ImageTransport &it);
  /**
   * @brief 初始化关键组件的一些参数和状态
   * 
   */
  void initializeComponents();
  /**
   * @brief 初始化保存输出文件相关的目录和文件
   * 
   */
  void initializeFiles();
  /**
   * @brief 主函数
   * 
   */
  void run();

  /**
   * @brief 重力对齐
   * 
   */
  void gravityAlignment();
  /**
   * @brief 处理第一帧数据
   * 
   */
  void handleFirstFrame();
  /**
   * @brief 进行LIO/VIO状态估计和地图构建
   * 
   */
  void stateEstimationAndMapping();
  /**
   * @brief 处理VIO流程
   * 
   */
  void handleVIO();
  /**
   * @brief 处理LIO流程
   * 
   */
  void handleLIO();
  /**
   * @brief 保存点云数据到PCD文件
   * 
   */
  void savePCD();
  /**
   * @brief 处理IMU数据
   * 
   */
  void processImu();
  
  /**
   * @brief 根据当前模式同步各类传感器数据
   * 
   * @param meas 
   * @return true 
   * @return false 
   */
  bool sync_packages(LidarMeasureGroup &meas);
  /**
   * @brief 根据IMU数据传播一次当前状态
   * 
   * @param imu_prop_state 
   * @param dt 
   * @param acc_avr 
   * @param angvel_avr 
   */
  void prop_imu_once(StatesGroup &imu_prop_state, const double dt, V3D acc_avr, V3D angvel_avr);
  /**
   * @brief IMU传播的定时函数
   * 
   * @param e 
   */
  void imu_prop_callback(const ros::TimerEvent &e);
  /**
   * @brief 根据R和t对雷达系下的点云做变换，R和t是T_world2imu，其中自动做了lidar2imu的变换，因为一般是在用lidar系的点云，转到world系下就用这个函数
   * 
   * @param rot 
   * @param t 
   * @param input_cloud 
   * @param trans_cloud 
   */
  void transformLidar(const Eigen::Matrix3d rot, const Eigen::Vector3d t, const PointCloudXYZI::Ptr &input_cloud, PointCloudXYZI::Ptr &trans_cloud);
  /**
   * @brief 将点从Body坐标系转换到World坐标系
   * 
   * @param pi 
   * @param po 
   */
  void pointBodyToWorld(const PointType &pi, PointType &po);
  /**
   * @brief 将RGB点从Body坐标系转换到World坐标系
   * 
   * @param pi 
   * @param po 
   */
  void RGBpointBodyToWorld(PointType const *const pi, PointType *const po);
  // 感觉上面两个函数可以合并成一个模板函数

  //********** 各类回调函数 **********//
  void standard_pcl_cbk(const sensor_msgs::PointCloud2::ConstPtr &msg);
  void livox_pcl_cbk(const livox_ros_driver::CustomMsg::ConstPtr &msg_in);
  void imu_cbk(const sensor_msgs::Imu::ConstPtr &msg_in);
  void img_cbk(const sensor_msgs::ImageConstPtr &msg_in);
  //********** 各类发布函数 **********//
  void publish_img_rgb(const image_transport::Publisher &pubImage, VIOManagerPtr vio_manager);
  void publish_frame_world(const ros::Publisher &pubLaserCloudFullRes, VIOManagerPtr vio_manager);
  void publish_visual_sub_map(const ros::Publisher &pubSubVisualMap);
  void publish_effect_world(const ros::Publisher &pubLaserCloudEffect, const std::vector<PointToPlane> &ptpl_list);
  void publish_odometry(const ros::Publisher &pubOdomAftMapped);
  void publish_mavros(const ros::Publisher &mavros_pose_publisher);
  void publish_path(const ros::Publisher pubPath);
  /**
   * @brief 从ROS参数服务器读取参数
   * 
   * @param nh 
   */
  void readParameters(ros::NodeHandle &nh);
  template <typename T> void set_posestamp(T &out);
  template <typename T> void pointBodyToWorld(const Eigen::Matrix<T, 3, 1> &pi, Eigen::Matrix<T, 3, 1> &po);
  template <typename T> Eigen::Matrix<T, 3, 1> pointBodyToWorld(const Eigen::Matrix<T, 3, 1> &pi);
  /**
   * @brief 将ROS消息中的图像转换为OpenCV的Mat格式
   * 
   * @param img_msg 
   * @return cv::Mat 
   */
  cv::Mat getImageFromMsg(const sensor_msgs::ImageConstPtr &img_msg);

  std::mutex mtx_buffer, mtx_buffer_imu_prop;
  std::condition_variable sig_buffer;

  SLAM_MODE slam_mode_;
  std::unordered_map<VOXEL_LOCATION, VoxelOctoTree *> voxel_map;
  
  string root_dir;
  string lid_topic, imu_topic, seq_name, img_topic;
  V3D extT;
  M3D extR;

  int feats_down_size = 0, max_iterations = 0;

  double res_mean_last = 0.05;
  double gyr_cov = 0, acc_cov = 0, inv_expo_cov = 0;
  double blind_rgb_points = 0.0;
  double last_timestamp_lidar = -1.0, last_timestamp_imu = -1.0, last_timestamp_img = -1.0;
  double filter_size_surf_min = 0;
  double filter_size_pcd = 0;
  double _first_lidar_time = 0.0;                 // 第一帧Lidar的时间戳，作为基准时间，用于计算时间偏移
  double match_time = 0, solve_time = 0, solve_const_H_time = 0;

  bool lidar_map_inited = false, pcd_save_en = false, pub_effect_point_en = false, pose_output_en = false, ros_driver_fix_en = false, hilti_en = false;
  int pcd_save_interval = -1, pcd_index = 0;
  int pub_scan_num = 1;

  StatesGroup imu_propagate, latest_ekf_state;

  // 下面四个变量都是用于IMU状态传播，发布高频里程计，一般情况用不上
  bool new_imu = false;                           // 是否有新的IMU数据
  bool state_update_flg = false;                  // 是否进行了状态更新
  bool imu_prop_enable = true;                    // 是否启用IMU状态传播
  bool ekf_finish_once = false;                   // 是否完成了一次EKF滤波过程
  
  deque<sensor_msgs::Imu> prop_imu_buffer;
  sensor_msgs::Imu newest_imu;
  double latest_ekf_time;
  nav_msgs::Odometry imu_prop_odom;
  ros::Publisher pubImuPropOdom;
  double imu_time_offset = 0.0;
  double lidar_time_offset = 0.0;

  bool gravity_align_en = false;                  // 是否启用重力对齐
  bool gravity_align_finished = false;            // 是否完成重力对齐

  bool sync_jump_flag = false;

  bool lidar_pushed = false, imu_en, gravity_est_en, flg_reset = false, ba_bg_est_en = true;
  bool dense_map_en = false;
  int img_en = 1, imu_int_frame = 3;
  bool normal_en = true;
  bool exposure_estimate_en = false;
  double exposure_time_init = 0.0;
  bool inverse_composition_en = false;
  bool raycast_en = false;
  int lidar_en = 1;
  bool is_first_frame = false;
  int grid_size, patch_size, grid_n_width, grid_n_height, patch_pyrimid_level;
  double outlier_threshold;
  double plot_time;
  int frame_cnt;
  double img_time_offset = 0.0;
  deque<PointCloudXYZI::Ptr> lid_raw_data_buffer;
  deque<double> lid_header_time_buffer;
  deque<sensor_msgs::Imu::ConstPtr> imu_buffer;
  deque<cv::Mat> img_buffer;
  deque<double> img_time_buffer;
  vector<pointWithVar> _pv_list;
  vector<double> extrinT;                         // lidar到IMU的平移外参 t_imu2lidar
  vector<double> extrinR;                         // lidar到IMU的旋转外参 R_imu2lidar
  vector<double> cameraextrinT;                   // lidar到相机的平移外参 t_camera2lidar
  vector<double> cameraextrinR;                   // lidar到相机的旋转外参 R_camera2lidar
  double IMG_POINT_COV;                           // 图像特征点协方差，即权重

  PointCloudXYZI::Ptr visual_sub_map;             // 视觉地图点云
  PointCloudXYZI::Ptr feats_undistort;            // 点云去畸变后、降采样前，在Lidar系下的点云
  PointCloudXYZI::Ptr feats_down_body;            // 点云降采样后，在Lidar系下的点云
  PointCloudXYZI::Ptr feats_down_world;           // 点云降采样后，在World系下的点云
  PointCloudXYZI::Ptr pcl_w_wait_pub;             // 一帧处理完之后的World系下用于发布的点云
  PointCloudXYZI::Ptr pcl_wait_pub;               // 发布函数中的一个临时变量
  PointCloudXYZRGB::Ptr pcl_wait_save;            // 用于保存的点云
  PointCloudXYZI::Ptr pcl_wait_save_intensity;    // 没图像时用于保存的强度点云

  ofstream fout_pre, fout_out, fout_pcd_pos, fout_points;

  pcl::VoxelGrid<PointType> downSizeFilterSurf;   // 点云降采样滤波器

  V3D euler_cur;                                  // 当前状态的旋转向量，由旋转矩阵转换而来

  LidarMeasureGroup LidarMeasures;                // 用于存放同步后传感器数据
  StatesGroup _state;                             // 当前状态量              
  StatesGroup  state_propagat;                    // IMU传播之后的状态量

  // ROS发布相关的一些对象
  nav_msgs::Path path;
  nav_msgs::Odometry odomAftMapped;
  geometry_msgs::Quaternion geoQuat;
  geometry_msgs::PoseStamped msg_body_pose;

  // 几个关键的类对象
  PreprocessPtr p_pre;                            // lidar预处理类
  ImuProcessPtr p_imu;                            // IMU处理类
  VoxelMapManagerPtr voxelmap_manager;            // 体素地图管理类
  VIOManagerPtr vio_manager;                      // VIO处理类

  ros::Publisher plane_pub;
  ros::Publisher voxel_pub;
  ros::Subscriber sub_pcl;
  ros::Subscriber sub_imu;
  ros::Subscriber sub_img;
  ros::Publisher pubLaserCloudFullRes;
  ros::Publisher pubNormal;
  ros::Publisher pubSubVisualMap;
  ros::Publisher pubLaserCloudEffect;
  ros::Publisher pubLaserCloudMap;
  ros::Publisher pubOdomAftMapped;
  ros::Publisher pubPath;
  ros::Publisher pubLaserCloudDyn;
  ros::Publisher pubLaserCloudDynRmed;
  ros::Publisher pubLaserCloudDynDbg;
  image_transport::Publisher pubImage;
  ros::Publisher mavros_pose_publisher;
  ros::Timer imu_prop_timer;                      // 定时器，用于IMU状态传播，用于给UAV提供高频里程计

  int frame_num = 0;                              // 帧数序号
  double aver_time_consu = 0;                     // 处理一帧整体的平均时间
  double aver_time_icp = 0;                       // lidar point2plane ICP平均时间
  double aver_time_map_inre = 0;                  // 地图更新平均时间
  bool colmap_output_en = false;
};
#endif