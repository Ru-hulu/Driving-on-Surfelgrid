#include<ros/ros.h>
#include <pcl/io/vtk_io.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/console/print.h>
#include <pcl/console/time.h>
#include <pcl/impl/pcl_base.hpp>
#include<iostream>
#include<string>
#include<sstream>
#include<fstream>
#include<ctime>
#include<cmath>
#include<ros/time.h>
#include <thread>
#include <Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <pcl/common/angles.h> // for pcl::deg2rad
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h> 
#include <pcl/console/parse.h>
#include <pcl/common/common_headers.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_ros/transforms.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>
#include <sensor_msgs/Image.h>
#include <gazebo_msgs/GetModelState.h>
#include <gazebo_msgs/SetModelState.h>
#include <tf/tf.h>
#include <map>
#include <set>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/tf.h>
#include <stdlib.h>
#include <nav_msgs/OccupancyGrid.h>
#include "Gridsurf.hpp"
#include <std_msgs/Bool.h>
#include"std_srvs/SetBool.h"
#include"opencv2/opencv.hpp"
#include <opencv2/core/mat.hpp>
#include <deque>

using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;
using namespace std::chrono_literals;


typedef union USHORT_UNION 
{
  ushort num;
  char c[2];
} ushort_union;

inline ushort Littlesw(char c0, char c1) 
{
  ushort_union tmp;
  tmp.c[0] = c0;
  tmp.c[1] = c1;
  return tmp.num;
}

class Teclas
{
  public:
  ros::NodeHandle nh;
  ros::Subscriber ims;
  ros::Publisher showp;
  ros::Publisher pubCloud;
  ros::Publisher showMap;
  ros::Publisher showone;
  ros::Publisher showLPS;
  ros::Publisher showLPS_VP;  
  ros::Publisher showSUBS;
  ros::Publisher showcenter;
  ros::Publisher gridmapPub;
  ros::Publisher planningspaceanchor_Pub;
  ros::Publisher planningspaceanchor_Pub1;
  ros::ServiceServer static_map_server;
  ros::ServiceServer map_server;
  ros::ServiceServer subspace_infor_server;
  ros::ServiceClient states_client;
  ros::ServiceClient initial_debug_client;
  ros::ServiceServer savetxt;
  ros::ServiceServer savedebug;
  bool image_in_process = true;//当前图像帧正在处理
  // ros::ServiceClient lcp_client;
  GridSurf* l_map;
  int map_v = 0;
  // std::set<octomap::OcTreeKey*,CompareKey> frontier;
  std::map<pos_xy,std::vector<float>,CompareKeyGrid> frontier;
  std::string static_map;
  Teclas(const ros::NodeHandle &nh_,std::string static_map_);
  int fit_plane_to_cloud(pcl::ModelCoefficients::Ptr coefficients,const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,const double dist_thes,float& precent_inline,float& varia);
  float K_inv[9] = {0.00143679,0,-0.919547,0,0.00143679,-0.517245,0,0,1};  
  float camera_depth=8;  
  float xita_thred=20.0/57.2957;//如果超过20度，则认为小平面不可穿行。
  float resolu=0.25;  float resolu_half=0.125;  
  std::deque<pcl::PointCloud<pcl::PointXYZ>::Ptr> point_deque;
  void insertCloudCallback(sensor_msgs::PointCloud2 in_cloud);
  void PublishPlane();//所有已知的平面
  void Point2SurfelMap(pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud);
  bool StaticMapService(std_srvs::SetBool::Request& req,std_srvs::SetBool::Response& res);
  bool SaveMap2txt(std_srvs::SetBool::Request& req,std_srvs::SetBool::Response& res);
  bool static compare_center(const std::pair<int, int>& a, const std::pair<int, int>& b); 
  // void LPS_Planner();//得到全局规划路径
  //点云转换的时间大概是70ms

  // void PublishCluster_Center();
  // void PublishPlane_USLESS();
  // void Project2D(int st_sub1,int st_sub2,float posx,float posy,float posz);
  // void Solve_ATSP(std::vector<std::pair<float,float>>* vp_l,Eigen::MatrixXf Weight_matrix);
  // bool inline CheckCollision(int grid2d_x,int grid2d_y,int grid_w,int grid_h);
  //LocalPlanningSpace中的所有frontier
  int cameracct=0; int frontier_filer_counter=0;
  std::vector<std::pair<int,int>> center2surround5;
  std::set<int> update_subspace_set;//深度相机一帧扫描到的Subspace
  int sub_num = 32;//32-32整张地图由多少个subspace构成
  int LPS_size = 3; //局部规划空间由3-3的subspace构成
  std::vector<std::pair<int,int>> checklist3;
  int ftcounter=0;
  int clear_marker_counter=0;
  int clear_marker_counter1=0;
  int finish_subspace_cct = 32*32;  
  visualization_msgs::MarkerArray PlaneMarker;//节点数据结构，用于可视化，记录所有的空间voxel

  static const int NOTUS = -2;
  static const int UNTRA = -1;
  static const int FRONT =  0;
  static const int TRAVE =  1;

  private:
    float map_min_x=1000000;    float map_min_y=1000000;
    float map_max_x=-1000000;    float map_max_y=-1000000;
    int g_w=1000;//400
    int g_h=1000;//400
    int sample_vp_ratio=4;
    float in_step = 1.0;//从frontier向内部缩进2m
    std::pair<int,int> now_subspace_center=std::make_pair<int,int>(-1,-1);
    nav_msgs::OccupancyGrid grid2dmap;
    nav_msgs::OccupancyGrid gdmap_show;
    std::vector<std::pair<float,float>> atsp_vp_sequence;
    std::vector<int> atsp_vp_sequence_index;
    std::vector<std::vector<geometry_msgs::PoseStamped>> atsp_vp_path_All;    
    std::vector<std::vector<geometry_msgs::PoseStamped>> atsp_vp_travel_path;    
};
