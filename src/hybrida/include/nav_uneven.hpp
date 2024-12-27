#include<ros/ros.h>
#include<iostream>
#include<string>
#include<fstream>
#include<ctime>
#include<cmath>
#include<ros/time.h>
#include <gazebo_msgs/GetModelState.h>
#include <gazebo_msgs/SetModelState.h>
#include <tf/tf.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <stdlib.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include "algorithm.hpp"
#include"std_srvs/SetBool.h"
#include"dubins.h"
#include"constants.h"
class Uneven_Navigator
{
    public:
        ros::NodeHandle nh;
        ros::Publisher org_pathpub;
        ros::Publisher opt_pathpub;
        ros::Publisher searchpub;
        ros::Publisher increpub;
        ros::Publisher MileStonepub;
        ros::ServiceServer navserver;
        Uneven_Navigator(const ros::NodeHandle &nh_,std::string map_string_);
        void Initstartend(float sxw,float syw,float szw,float syaw,
        float txw,float tyw,float tzw,float tyaw);
        bool NavService(std_srvs::SetBool::Request& req,std_srvs::SetBool::Response& res);
        Surfel*** InitialMap();
        void ShowTraj();
        void VisualMark(float w_x,float w_y, float w_z,float yaw);
        std::vector<std::pair<int,int>> valid_key;
        nav_msgs::Path origin_path;
        nav_msgs::Path une_opt_path;
        visualization_msgs::MarkerArray vMarker_arr;
        int vmcct = 0;
        int width_x;
        int heigh_y;
        float start_x, start_y, start_z, start_yaw;
        float target_x, target_y, target_z, target_yaw;
        std::string map_string;
        
                std::string result_path;
                double org_path_search_time = 0;
                double opt_path_search_time = 0;
                double path_len_opt = 0;
                double path_smooth_cost = 0;
                double path_max_acc = 0;
                double path_max_curvature = 0;
                std::vector<Eigen::Vector3d> traj_opt;
};