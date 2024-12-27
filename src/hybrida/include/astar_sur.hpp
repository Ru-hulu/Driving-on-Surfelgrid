#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <fstream>
#include <iostream>
#include <nlopt.hpp>
#include <vector>
#include <set>
#include <cmath>
#include <queue>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <queue>
#include<stdint.h>
#include<algorithm>
#include"Surf.hpp"
#include <Eigen/Core>
#include <eigen3/Eigen/Dense> 
//专门用来做导航的点，和地图的数据结构无关。
struct Surf3d
{
  int key_x;
  int key_y;
  int key_z;
  int cost;
  int g;
  int h;
  float fz;
  bool is_open = false;
  Surf3d* parent;
  Surf3d():key_x(0),key_y(0),key_z(0),parent(NULL) {}
};
    
struct Comparexyz 
{
  bool operator()(Surf3d* lhs, Surf3d* rhs) const 
  {
    if (lhs->key_x != rhs->key_x) return lhs->key_x < rhs->key_x;
    if (lhs->key_y != rhs->key_y) return lhs->key_y < rhs->key_y;
                                  return lhs->key_z < rhs->key_z;
  }
};

    struct Comparecost 
    {
      bool operator()(Surf3d* lhs, Surf3d* rhs) const 
      {
        return lhs->cost > rhs->cost;
      }//这样反而会从小到大排序
    };

class AstarSurPlanner
{
  public:
      Surfel*** MultiLayer_Grid;
      AstarSurPlanner(Surfel*** MultiLayer_Grid_,int width_x_,int heigh_y_,float rz_,ros::Publisher & in_pub);
      float AstarPlan(float start_x, float start_y, float start_z, float target_x, float target_y, float target_z);   
      int GetMilestones(float start_x, float start_y, float start_z, float start_yaw, 
      float target_x, float target_y, float target_z, float target_yaw, 
      std::vector<std::vector<float>>& milestone);
      Eigen::Vector3f computePCA(const std::vector<Eigen::Vector3f>& points);
      inline bool check_safe(Surf3d* temp_neibour);
      int map_width_x = 0;
      int map_heigh_y = 0;
      float max_milestone_step = 10;
      float rz;
      int path_range_min_x = 0;
      int path_range_min_y = 0;
      int path_range_max_x = 0;
      int path_range_max_y = 0;
      // std::vector<geometry_msgs::PoseStamped> plan;//只有路径的空间坐标
      std::vector<std::pair<int,int>> xy_patch = {{-1,0},{1,0},{0,1},{0,-1},
      {-1,-1},{-1,1},{1,-1},{1,1}};
      int check_safe_size = 2;//2 voxel
      std::vector<int>cost_move = {10,10,10,10,14,14,14,14};
      ros::Publisher test_pub;
      visualization_msgs::MarkerArray allMarker;
};
 