#include"dubins.h"
#include"node3d.hpp"
#include"astar_sur.hpp"
#include "non_uniform_bspline.hpp"
#include<set>
#include<deque>
#include<unordered_map>
#include<ros/ros.h>
#include<tf/tf.h>
#include <list>
#include <Eigen/Core>
#include <eigen3/Eigen/Dense>
// #include <ompl/base/spaces/ReedsSheppStateSpace.h>
// #include <ompl/base/spaces/DubinsStateSpace.h>
// #include <ompl/base/spaces/SE2StateSpace.h>
// #include <ompl/base/State.h>
// typedef ompl::base::SE2StateSpace::StateType State;

namespace HybridAStar 
{
   const int   dir = 6; //三个方向。注意2D Node为8个

   // const float dx[] = { 0.873,  0.866115935, 0.866115935};//直行时，前进0.873个grid，即0.21825m,如果在圆弧上，对应0.21825弧度，此时x前进为sin(0.21825)*半径=0.216528984m,转换为0.866115935grid
   // const float dy[] = { 0,      0.094888311,-0.094888311};//此时y前进(1-cos(0.21825))*半径 = 0.023722078m，转换为0.094888311grid
   // const float dt[] = { 0,      0.21825,    -0.21825};//0.21825弧度 > 5 度
   //按照半径为1 确定的参数
   //请注意,这里所有的计算单位都是grid,不是M!!!
   const float dx[] = { 0.873,  0.871268689, 0.871268689};//直行时，前进0.873个grid，即0.21825m,如果在圆弧上，对应0.109125弧度，此时x前进为sin(0.109125)*半径(2)=0.217817172m,转换为0.871268689grid
   const float dy[] = { 0,      0.047585846,-0.047585846};//此时y前进(1-cos(0.109125))*2 = 0.011896462m，转换为0.047585846grid
   const float dt[] = { 0,      0.109125,    -0.109125};//0.109125弧度 > 5 度
   //请注意,这里所有的计算单位都是grid,不是M!!!

   const float yaw_res = 0.087266;//这里是按照5度计算，与半径无关.
   const float sam_z_thr = 1.0;
   const int NOTUS = -2;
   const int UNTRA = -1;
   const int FRONT =  0;
   const int TRAVE =  1;
   const int MAXITERATION = 5000;
   const std::vector<std::pair<float,float>> nei_p = {{1,0},{0,1},{1,1},
                                                      {-1,0},{0,-1},{-1,-1},
                                                      {1,-1},{-1,1},{0,0}};


struct CompareNode3d
{
   bool operator()(const Node3D* n1,const Node3D* n2)
   {
      if(n1->grid_x<n2->grid_x)return true;
      else if(n1->grid_x==n2->grid_x && n1->grid_y<n2->grid_y)return true;
      else if(n1->grid_x==n2->grid_x && n1->grid_y==n2->grid_y && n1->yaw_index<n2->yaw_index)return true;
      else return false; 
   }
};
struct Comparevalue
{
   // bool operator()(const Node3D* n1,const Node3D* n2)
   // {
   //    int cost_1 = (n1->g+n1->h)*10000;
   //    int cost_2 = (n2->g+n2->h)*10000;

   //    if(cost_1<cost_2)return true;
   //    else if((cost_1==cost_2) && (n1->grid_x<n2->grid_x))return true;
   //    else if((cost_1==cost_2) && (n1->grid_x==n2->grid_x) && (n1->grid_y<n2->grid_y))return true;
   //    else if((cost_1==cost_2) && (n1->grid_x==n2->grid_x) && (n1->grid_y==n2->grid_y)
   //    &&(n1->yaw_index<n2->yaw_index))return true;
   //    return false; 
   // }
   //希望决策不仅能够考虑到总路径，而且能够再
   //总路径和h的矛盾，随着h减小，总路径并没有显著的降低。
   //所以我们希望在 保证总路径启发的前提下，倾向性的使用h启发

   bool operator()(const Node3D* n1,const Node3D* n2)
   {
      int cost_1 = (n1->g+n1->h)*100;
      int cost_2 = (n2->g+n2->h)*100;

      if(cost_1 < cost_2)return true;
      else if(( n1->h   <  n2->h)  && 
              (cost_1  ==  cost_2)) return true;
      else if((cost_1  ==  cost_2) && 
              ( n1->h  ==  n2->h) &&
         (n1->grid_x    <  n2->grid_x))return true;
      else if((cost_1  ==  cost_2) && 
              (n1->h   ==  n2->h) &&
         (n1->grid_x   ==  n2->grid_x) && 
         (n1->grid_y    <  n2->grid_y))return true;
      else if((cost_1  ==  cost_2) && 
              (n1->h   ==  n2->h) &&
         (n1->grid_x   ==  n2->grid_x) && 
         (n1->grid_y   ==  n2->grid_y) &&
      (n1->yaw_index    <  n2->yaw_index))return true;
      else if((cost_1  ==  cost_2) && 
              ( n1->h  ==  n2->h) &&
          (n1->grid_x  ==  n2->grid_x) && 
          (n1->grid_y  ==  n2->grid_y) &&
       (n1->yaw_index  ==  n2->yaw_index) &&
         (n1->z_layer   <  n2->z_layer) )return true;
      return false; 
   }
};
struct Comparepair
{
   bool operator()(std::pair<int,int> p1,std::pair<int,int> p2)
   {
      if(p1.first<p2.first)return true;
      else if(p1.first==p2.first && p1.second<p2.second)return true;
      return false;
   }
};
typedef std::set<Node3D*,HybridAStar::CompareNode3d>::iterator find_itt;
typedef std::set<Node3D*,HybridAStar::Comparevalue>::iterator open_itt;

struct pointlmr
{
    float x;
    float y;
    float z;
    float xita;
    float k;
    pointlmr* l_;
    pointlmr* r_;
    bool valid_flag = true;    
    pointlmr():x(0),y(0),z(0),xita(0),k(0),l_(nullptr),r_(nullptr),valid_flag(false){};
};

class Algorithm 
{
 public:
   Algorithm(int width_x_, int heigh_y_, float res_, Surfel*** MultiLayer_Grid_,ros::Publisher& tp_b_
   ,ros::Publisher& tp_b1_);
   int width_x;
   int heigh_y;
   int grid_dubin_expand = 6;
   float res;
   Surfel*** MultiLayer_Grid;
   ros::Publisher tp_b;
   ros::Publisher tp_b1;
   std::set<Node3D*,CompareNode3d> find_set;
   std::set<Node3D*,Comparevalue> open_list;
   std::vector<Eigen::Vector3d> spline_result;
   void VisualMarkwithscore();
   void VisualTra(Node3D* thisn);
   void VisualIncre(Node3D* thisn,bool isc);
   Node3D* createSucc(Node3D* this_node,float a,float b,float c,float d,int prim_index);
   Node3D* hybridAStar(float start_x,float start_y,float start_z,float start_yaw,float target_x,float target_y,float target_z,float target_yaw);
   bool PrepareNavi(float now_x,float now_y,float now_z,float now_yaw,
   float target_x,float target_y,float target_z,float target_yaw,vector<vector<float>>& milestone);
   std::pair<Node3D*,Node3D*> dubinsShot(float now_x,float now_y,float now_z,float now_yaw,float target_x,float target_y,float target_z,float target_yaw);
   inline void Coor2KeyXY(float w_x,float w_y,int& key_x,int& key_y);
   //在地图中进行搜索节点，如果地图中有这个节点则返回key，如果不存在节点返回false
   bool Coor2KeyXYZYaw(float w_x,float w_y,float w_z,float w_yaw,int& key_x,int& key_y,int& key_z,int& key_yaw,float& proj_z);
   inline void Key2Coor(int kx,int ky,float& coorx,float& coory);
   bool isreach(Node3D* this_node,int target_k_x,int target_k_y,float target_z,int target_k_yaw);
   Surfel* SearchinMap(float w_x,float w_y,float w_z,int& z_layer);
   void ClearFrontDataSet();
   bool CheckTraversable(Node3D* this_node);
   //返回值以grid作为单位
   float CaculateH(float now_x,float now_y,float now_z,float now_yaw,float target_x,float target_y,float target_z,float target_yaw);
   //返回值以grid作为单位
   float CaculateG(float pre_g,int pre_prim,int now_prim);
   inline bool Check_Four_Wheels(float w_x,float w_y,float w_z,float w_yaw);
   std::pair<bool,float> Caculate_z(float w_x,float w_y,float fuzzy_z);
   inline float MoveCost(Node3D* node1,Node3D* node2);
   inline bool SameKey(Node3D* node1,Node3D* node2);
   void VisualMark(float w_x,float w_y, float w_z,float yaw,int prim_t);
   void VisualPoly(float x1,float y1,float z1,
                   float x2,float y2,float z2,
                   float w_x,float w_y,float w_z,
                   float color_r,float color_g,float color_b);
   visualization_msgs::MarkerArray allMarker;//节点数据结构，用于可视化，记录所有的空间voxel allMarker;//节点数据结构，用于可视化，记录所有的空间voxel
   visualization_msgs::MarkerArray increMarker;//节点数据结构，用于可视化，记录所有的空间voxel
                  //这段算法是自己提出的，在非平坦环境中进行优化的策略
                  void UNE_Prepare_Obs();
                  void UNE_Opt_Traj();          
                  void UNE_Time_Adjust();
                  void UNE_opt();
                  void Uev_GetStepPath(Node3D* start_node);
                  static double costFunction(const std::vector<double> &x, std::vector<double>& grad ,void* func_data);
                  void Bspline_Fit();
                  std::vector<std::vector<float>> une_tra_data;//初始化优化前的轨迹
                  std::vector<std::vector<float>> une_tra_data_opt;//优化后的数据
                  std::vector<int> must_node;//不允许动的路径点
                  std::vector<Surfel*> path_local_geometry;//用于凸包生成
                  std::vector<std::pair<Eigen::Vector3f,Eigen::Vector3f>> nei_obs_list;//用于凸包生成
                  std::vector<Eigen::Matrix4f>une_tra_planes_Twp;//Twp 
                  std::vector<Eigen::Matrix4f>une_tra_planes_Tpw;//Tpw 
                  std::vector<std::vector<Eigen::Vector3f>>une_tra_constraint;//这里记录每个采样点对应的凸包约束
                  float une_obs_loss_w = 0.08;
                  float une_smo_loss_w = 0.90;
                  float une_fea_loss_w = 0.02;
                  float opt_bound = 0.4;// the boundary of optimization
                  int   une_opt_iteration = 500;
                  int opt_cct = 0;
                  float min_loss = 100000;
                  int obs_ext_dis = 1.0;// The distance we used to extract the nearest obstacle. 
                  float obs_rs = 0.12;
                  float obs_safty_th = 0.5;
                  float max_max_velocity = 1.0;
                  int step_path_cct = 0;

                  double prepare_time = 0;
                  double org_path_search_time = 0;
                  double opt_path_search_time = 0;
                  double path_len_opt = 0;
                  double path_smooth_cost = 0;
                  double path_max_acc = 0;
                  double path_max_curvature = 0;

                  //这段算法是自己提出的，在非平坦环境中进行优化的策略
                  //这段算法是复现driving on pointclouds 论文的策略，所有实现都放在graph_opt中
                  int cctt = 0;
                  int ccincre = 0; 
               private:
                  std::ifstream pcl_traj_file; // 类的成员对象
                  float car_size_x_2 = 0.35;
                  float car_size_y_2 = 0.25;
                  float fourwheel_error = 0.1;
};
}
//计算节点在空间中左右偏移的坐标、角度、曲率
//构建图搜索结构
//根据图搜索结构计算路径损失，得到最小点