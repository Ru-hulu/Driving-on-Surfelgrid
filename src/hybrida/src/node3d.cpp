#include "node3d.hpp"
Node3D::Node3D(int grid_x_,int grid_y_,int z_layer_,int yaw_index_,int prim_,float float_x_,float float_y_,float float_z_,float t_)
{
    grid_x = grid_x_;
    grid_y = grid_y_;
    z_layer = z_layer_;
    yaw_index = yaw_index_;
    g = 0;//目前代价值
    h = 0;//启发式值
    is_open = true;//是否属于open
    prim = prim_;
    float_x = float_x_;
    float_y = float_y_;
    float_z = float_z_;
    t = t_;
    parent = nullptr;
};
