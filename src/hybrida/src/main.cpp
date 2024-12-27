#include <cstring>
#include <iostream>
#include <ros/ros.h>
#include "constants.h"
#include "nav_uneven.hpp"
#include <Eigen/Core>
//功能：
//输入：自定义srv获得地图信息和当前位置、目标位置
//输出：参考坐标、参考指令通过另外一个srv发出给Controller。
 

int main(int argc, char** argv) 
{
    ros::init(argc, argv, "uneven_nav");
    ros::NodeHandle nh;
    float start_x;     
    float start_y; 
    float start_z; 
    float start_yaw; 
    
    float target_x;      
    float target_y;    
    float target_z;    
    float target_yaw; 
    std::string map_path;
    nh.getParam("start_x", start_x);
    nh.getParam("start_y", start_y);
    nh.getParam("start_z", start_z);
    nh.getParam("start_yaw", start_yaw);

    nh.getParam("target_x", target_x);
    nh.getParam("target_y", target_y);
    nh.getParam("target_z", target_z);
    nh.getParam("target_yaw", target_yaw);

    nh.getParam("map_path", map_path);
    std::string result_path;
    nh.getParam("result_path", result_path);

    std::cout<<"get param"<<std::endl;
    std::cout<<start_x<<" "<<start_y<<" "<<start_z<<" "<<start_yaw<<std::endl;
    std::cout<<target_x<<" "<<target_y<<" "<<target_z<<" "<<target_yaw<<std::endl;
    std::cout<<"get param"<<std::endl;


    Uneven_Navigator uneven_nav_handle(nh,map_path);
    uneven_nav_handle.result_path = result_path;
    uneven_nav_handle.Initstartend(start_x,start_y,start_z,start_yaw,
    target_x,target_y,target_z,target_yaw);
    ros::spin();
    return 0;
}