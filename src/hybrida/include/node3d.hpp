#include <cmath>
#include "constants.h"
#include "helper.h"
//EXAMPLE grid 分辨率为0.1,转弯半径为1
// const float Node3D::dx[] = { 0.873,  0.8718903,    0.8718903};//直行时，前进0.873个grid，即0.0873m,如果在圆弧上，对应0.0873弧度，此时x前进为sin(0.0873)*半径=0.087189032m,转换为0.87189032grid
// const float Node3D::dy[] = { 0,       0.03808215,-0.03808215};//此时y前进(1-cos(0.0873))*半径 = 0.003808215m，转换为0.03808215grid
// const float Node3D::dt[] = { 0,       0.0873,   -0.0873};//0.0873弧度 > 5 度
//注意：数组中的单位是grid
//grid 分辨率为0.25，转弯半径为1
class Node3D 
{//所有信息都以grid为单位
  public:
  Node3D():parent(nullptr){};
  Node3D(int grid_x_,int grid_y_,int z_layer_,int yaw_index_,int prim_,float float_x_,float float_y_,float float_z_,float t_);
  int grid_x; //单位是grid 
  int grid_y; //单位是grid 
  int z_layer; //单位是grid 
  int yaw_index;
  float g;//目前代价值
  float h;//启发式值
  bool is_open;//是否属于open
  int prim;
  float float_x;
  float float_y;
  float float_z;
  float t;//就是yaw角
  Node3D* parent;//祖先节点
};