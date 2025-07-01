/* 
This file is part of FAST-LIVO2: Fast, Direct LiDAR-Inertial-Visual Odometry.

Developer: Chunran Zheng <zhengcr@connect.hku.hk>

For commercial use, please contact me at <zhengcr@connect.hku.hk> or
Prof. Fu Zhang at <fuzhang@hku.hk>.

This file is subject to the terms and conditions outlined in the 'LICENSE' file,
which is included as part of this source code package.
*/

#ifndef VIO_H_
#define VIO_H_

#include "voxel_map.h"
#include "feature.h"
#include <opencv2/imgproc/imgproc_c.h>
#include <pcl/filters/voxel_grid.h>
#include <set>
#include <vikit/math_utils.h>
#include <vikit/robust_cost.h>
#include <vikit/vision.h>
#include <vikit/pinhole_camera.h>

struct SubSparseMap
{
  vector<float> propa_errors;
  vector<float> errors;
  vector<vector<float>> warp_patch;
  vector<int> search_levels;
  vector<VisualPoint *> voxel_points;
  vector<double> inv_expo_list;
  vector<pointWithVar> add_from_voxel_map;

  SubSparseMap()
  {
    propa_errors.reserve(SIZE_LARGE);
    errors.reserve(SIZE_LARGE);
    warp_patch.reserve(SIZE_LARGE);
    search_levels.reserve(SIZE_LARGE);
    voxel_points.reserve(SIZE_LARGE);
    inv_expo_list.reserve(SIZE_LARGE);
    add_from_voxel_map.reserve(SIZE_SMALL);
  };

  void reset()
  {
    propa_errors.clear();
    errors.clear();
    warp_patch.clear();
    search_levels.clear();
    voxel_points.clear();
    inv_expo_list.clear();
    add_from_voxel_map.clear();
  }
};

class Warp
{
public:
  Matrix2d A_cur_ref;
  int search_level;
  Warp(int level, Matrix2d warp_matrix) : search_level(level), A_cur_ref(warp_matrix) {}
  ~Warp() {}
};

class VOXEL_POINTS
{
public:
  std::vector<VisualPoint *> voxel_points;
  int count;
  VOXEL_POINTS(int num) : count(num) {}
  ~VOXEL_POINTS() 
  { 
    for (VisualPoint* vp : voxel_points) 
    {
      if (vp != nullptr) { delete vp; vp = nullptr; }
    }
  }
};

class VIOManager
{
public:
  int grid_size;                                            // 图像网格大小，
  vk::AbstractCamera *cam;
  vk::PinholeCamera *pinhole_cam;
  StatesGroup *state;                                       // 与LIO中共用一个状态
  StatesGroup *state_propagat;
  M3D Rli, Rci, Rcl, Rcw, Jdphi_dR, Jdp_dt, Jdp_dR;
  V3D Pli, Pci, Pcl, Pcw;
  vector<int> grid_num;
  vector<int> map_index;
  vector<int> border_flag;
  vector<int> update_flag;
  vector<float> map_dist;
  vector<float> scan_value;
  vector<float> patch_buffer;
  bool normal_en, inverse_composition_en, exposure_estimate_en, raycast_en, has_ref_patch_cache;
  bool ncc_en = false, colmap_output_en = false;

  int width;                                                // 图像宽度
  int height;                                               // 图像高度
  int grid_n_width;                                         // 图像宽度方向的网格数量
  int grid_n_height;                                        // 图像高度方向的网格数量，17
  int length;                                               // 图像网格总数，grid_n_width * grid_n_height
  double image_resize_factor;
  double fx, fy, cx, cy;
  int patch_pyrimid_level;                                  // 图像金字塔层数，3
  int patch_size;                                           // 图像块大小，单位像素，8
  int patch_size_total;                                     // 图像块像素总量，8x8 = 64
  int patch_size_half;                                      // 图像块半宽，4  
  int border;                                               // 图像边界，40
  int warp_len;                                             // 64x3 = 192
  int max_iterations, total_points;                         // 最大迭代次数，从视觉地图中提取地图点的数量

  double img_point_cov, outlier_threshold, ncc_thre;
  
  SubSparseMap *visual_submap;                              // 当前帧的视觉子图
  std::vector<std::vector<V3D>> rays_with_sample_points;    // 图像各个网格在网格中心的光线上的距离相同步长的采样点

  double compute_jacobian_time, update_ekf_time;
  double ave_total = 0;
  // double ave_build_residual_time = 0;
  // double ave_ekf_time = 0;

  int frame_count = 0;
  bool plot_flag;

  Matrix<double, DIM_STATE, DIM_STATE> G, H_T_H;
  MatrixXd K, H_sub_inv;

  ofstream fout_camera, fout_colmap;
  unordered_map<VOXEL_LOCATION, VOXEL_POINTS *> feat_map;   // 视觉地图
  unordered_map<VOXEL_LOCATION, int> sub_feat_map;          // 当前帧点云的一个体素地图
  unordered_map<int, Warp *> warp_map;                      // 仿射矩阵的缓存
  vector<VisualPoint *> retrieve_voxel_points;              // 从视觉地图中提取的地图点
  vector<pointWithVar> append_voxel_points;                 // 待加入视觉地图的点云
  FramePtr new_frame_;                                      // 当前帧 
  cv::Mat img_cp, img_rgb, img_test;

  enum CellType
  {
    TYPE_MAP = 1,
    TYPE_POINTCLOUD,
    TYPE_UNKNOWN
  };

  VIOManager();
  ~VIOManager();
  /**
   * @brief 类似于逆向光流法，只计算一次参考帧patch的雅克比，这里没优化曝光时间
   * 
   * @param img 
   * @param level 
   */
  void updateStateInverse(cv::Mat img, int level);
  /**
   * @brief 更新步的函数
   * 
   * @param img 
   * @param level 
   */
  void updateState(cv::Mat img, int level);
  /**
   * @brief VIO处理的主函数
   * 
   * @param img 
   * @param pg world系下带协方差的点云
   * @param feat_map 当前的voxelmap
   * @param img_time 没用到
   */
  void processFrame(cv::Mat &img, vector<pointWithVar> &pg, const unordered_map<VOXEL_LOCATION, VoxelOctoTree *> &feat_map, double img_time);
  /**
   * @brief 类似于1中的addFromSparseMap
   * 
   * @param img 
   * @param pg world系下带协方差的点云
   * @param plane_map 当前的voxelmap
   */
  void retrieveFromVisualSparseMap(cv::Mat img, vector<pointWithVar> &pg, const unordered_map<VOXEL_LOCATION, VoxelOctoTree *> &plane_map);
  /**
   * @brief 将lidar点云加入视觉地图中
   * 
   * @param img 
   * @param pg 
   */
  void generateVisualMapPoints(cv::Mat img, vector<pointWithVar> &pg);
  /**
   * @brief 设置imu到lidar的外参，T_lidar_imu
   * 
   * @param transl 
   * @param rot 
   */
  void setImuToLidarExtrinsic(const V3D &transl, const M3D &rot);
  /**
   * @brief 设置lidar到相机的外参，T_cam_lidar
   * 
   * @param R 
   * @param P 
   */
  void setLidarToCameraExtrinsic(vector<double> &R, vector<double> &P);
  /**
   * @brief 初始化VIO
   * 
   */
  void initializeVIO();
  /**
   * @brief 用双线性插值从图像中提取浮点像素坐标的图像块
   * 
   * @param img 
   * @param pc 
   * @param patch_tmp 
   * @param level 
   */
  void getImagePatch(cv::Mat img, V2D pc, float *patch_tmp, int level);
  /**
   * @brief 计算投影点对三维点的雅可比矩阵
   * 
   */
  void computeProjectionJacobian(V3D p, MD(2, 3) & J);
  /**
   * @brief 计算光度误差对状态的雅可比矩阵，更新ieskf
   * 
   * @param img 
   */
  void computeJacobianAndUpdateEKF(cv::Mat img);
  /**
   * @brief 重置图像grid相关变量
   * 
   */
  void resetGrid();
  /**
   * @brief 给视觉地图点添加新的观测
   * 
   * @param img 
   */
  void updateVisualMapPoints(cv::Mat img);
  /**
   * @brief 计算仿射变换矩阵，假设图像块对应的三维空间表面是一个正对着参考相机的平面，更简单，不需要法向量信息，是对真实情况的一种近似
   * 
   * @param cam 
   * @param px_ref 
   * @param f_ref 
   * @param depth_ref 
   * @param T_cur_ref 
   * @param level_ref 
   * @param pyramid_level 
   * @param halfpatch_size 
   * @param A_cur_ref 仿射变换矩阵，从参考帧变换至当前帧，A_cur2ref
   */
  void getWarpMatrixAffine(const vk::AbstractCamera &cam, const Vector2d &px_ref, const Vector3d &f_ref, const double depth_ref, const SE3 &T_cur_ref,
                           const int level_ref, 
                           const int pyramid_level, const int halfpatch_size, Matrix2d &A_cur_ref);
  /**
   * @brief 计算仿射变换矩阵，假设图像块对应的三维空间表面是一个任意朝向的平面，使用更通用的单应性（Homography）模型
   * 
   * @param cam 
   * @param px_ref 
   * @param xyz_ref 
   * @param normal_ref 
   * @param T_cur_ref 
   * @param level_ref 
   * @param A_cur_ref 仿射变换矩阵，从参考帧变换至当前帧，A_cur2ref
   */
  void getWarpMatrixAffineHomography(const vk::AbstractCamera &cam, const V2D &px_ref,
                                     const V3D &xyz_ref, const V3D &normal_ref, const SE3 &T_cur_ref, const int level_ref, Matrix2d &A_cur_ref);
  /**
   * @brief 将参考帧的图像块进行仿射变换，得到当前帧的图像块
   * 
   * @param A_cur_ref 
   * @param img_ref 
   * @param px_ref 
   * @param level_ref 
   * @param search_level 
   * @param pyramid_level 
   * @param halfpatch_size 
   * @param patch 
   */
  void warpAffine(const Matrix2d &A_cur_ref, const cv::Mat &img_ref, const Vector2d &px_ref, const int level_ref, const int search_level,
                  const int pyramid_level, const int halfpatch_size, float *patch);
  /**
   * @brief 往视觉地图中添加一个新的地图点
   * 
   * @param pt_new 
   */
  void insertPointIntoVoxelMap(VisualPoint *pt_new);
  /**
   * @brief 画出当前帧跟踪的地图点
   * 
   */
  void plotTrackedPoints();
  /**
   * @brief 根据输入状态量更新当前帧的状态
   * 
   * @param state 
   */
  void updateFrameState(StatesGroup state);
  /**
   * @brief 调试用的函数，将参考帧patch转换至当前帧并做可视化
   * 
   * @param plane_map 
   */
  void projectPatchFromRefToCur(const unordered_map<VOXEL_LOCATION, VoxelOctoTree *> &plane_map);
  /**
   * @brief 当地图点的观测足够多时，更新地图点的法向量和参考帧
   * 
   * @param plane_map 
   */
  void updateReferencePatch(const unordered_map<VOXEL_LOCATION, VoxelOctoTree *> &plane_map);
  /**
   * @brief 计算参考帧的patch的雅克比
   * 
   * @param level 
   */
  void precomputeReferencePatches(int level);
  /**
   * @brief colmap格式数据保存
   * 
   */
  void dumpDataForColmap();
  /**
   * @brief 计算NCC
   * 
   * @param ref_patch 
   * @param cur_patch 
   * @param patch_size 
   * @return double 
   */
  double calculateNCC(float *ref_patch, float *cur_patch, int patch_size);
  /**
   * @brief 根据仿射变换矩阵的行列式确定最佳的金字塔搜索层级
   * 
   * @param A_cur_ref 
   * @param max_level 
   * @return int 
   */
  int getBestSearchLevel(const Matrix2d &A_cur_ref, const int max_level);
  /**
   * @brief 用双线性插值得到图像的浮点像素坐标处的RGB值，用于点云作色
   * 
   * @param img 
   * @param pc 
   * @return V3F 
   */
  V3F getInterpolatedPixel(cv::Mat img, V2D pc);
  
  // void resetRvizDisplay();
  // deque<VisualPoint *> map_cur_frame;
  // deque<VisualPoint *> sub_map_ray;
  // deque<VisualPoint *> sub_map_ray_fov;
  // deque<VisualPoint *> visual_sub_map_cur;
  // deque<VisualPoint *> visual_converged_point;
  // std::vector<std::vector<V3D>> sample_points;

  // PointCloudXYZI::Ptr pg_down;
  // pcl::VoxelGrid<PointType> downSizeFilter;
};
typedef std::shared_ptr<VIOManager> VIOManagerPtr;

#endif // VIO_H_