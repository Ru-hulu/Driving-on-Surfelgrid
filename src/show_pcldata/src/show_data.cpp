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
#include <sensor_msgs/Image.h>
#include <gazebo_msgs/GetModelState.h>
#include <gazebo_msgs/SetModelState.h>
#include <tf/tf.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
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
#include <std_msgs/Bool.h>
#include <pcl/filters/passthrough.h>
#include <deque>




#include<iostream>
#include<string>
#include <stdlib.h>
#include <stdio.h>
#include <sstream>
#include <vector>
 
#include<ros/ros.h>  
#include<pcl/point_cloud.h>  
#include<pcl_conversions/pcl_conversions.h>  
#include<sensor_msgs/PointCloud2.h>  
#include<pcl/io/pcd_io.h>
 
#include <octomap_msgs/OctomapWithPose.h> 
#include <octomap_msgs/conversions.h> 
#include <octomap_msgs/Octomap.h>
#include <octomap/octomap.h>
struct P 
{
    int x;
    int y;
    int z;
};
 
class tmp
{
    public:
        bool operator()(P a,P b)
        {
            if(a.x<b.x)return true;
            else if(a.x==b.x && a.y<b.y) return true;
            else if(a.x==b.x && a.y==b.y && a.z<b.z) return true;
            return false;  
        }
};
std::set<P,tmp> all_voxel;
int cct = 0;
float rz = 0.1;
void GetArray(visualization_msgs::MarkerArray &mkar)
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time::now();
  marker.ns = "now_search";
  marker.type = visualization_msgs::Marker::CUBE;
  marker.action = visualization_msgs::Marker::ADD;

  marker.color.a = 1.0;
  marker.lifetime = ros::Duration(1.0);/////marker的生存时间只有1s
  marker.scale.x = rz;
  marker.scale.y = rz;
  marker.scale.z = rz;
      marker.color.r = 1.0;
      marker.color.g = 1.0;
      marker.color.b = 1.0;

    mkar.markers.clear();
    for(P p:all_voxel)
    {
        if(cct>20000)break;
        cct++;
        marker.id = cct;
        marker.pose.position.x = float(p.x) * rz;
        marker.pose.position.y = float(p.y) * rz;
        marker.pose.position.z = float(p.z) * rz;
        mkar.markers.push_back(marker);
    }
}
int main(int argc, char** argv) 
{
  ros::init(argc, argv, "pcl_publisher");
  ros::NodeHandle nh;
  ros::Publisher pcl_pub = nh.advertise<sensor_msgs::PointCloud2>("/points", 10);
  ros::Publisher showp = nh.advertise<visualization_msgs::MarkerArray>("point_Marker",5);  
  visualization_msgs::MarkerArray mkar;
  std::ifstream file("/home/r/Mysoftware/ExplorationUneven/chapt2_simulation/chapt2_scene3_densepcl.txt"); 
  if (!file.is_open()) {
    ROS_ERROR("Failed to open file");
    return -1;
  }
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_data(new pcl::PointCloud<pcl::PointXYZ>);
  octomap::OcTree octomap_map(0.04); // 设置Octomap地图的分辨率为0.1米

  ros::Publisher octomap_pub = nh.advertise<octomap_msgs::Octomap>("/octomap", 1);
  std::string line;
  double max_z=-10000;
  double min_z=10000;
  while (std::getline(file, line)) {
    std::istringstream iss(line);
    std::vector<double> all_d;
    std::string ts;
    while(getline(iss,ts,' '))
    {
        all_d.push_back(std::stod(ts));        
    }
    
    pcl_data->push_back(pcl::PointXYZ(float(all_d[0]), float(all_d[1]), float(all_d[2])));
    min_z = std::min(min_z,all_d[2]);
    max_z = std::max(max_z,all_d[2]);
    struct P thisv;

    octomap_map.updateNode(all_d[0],all_d[1],all_d[2], true);

    thisv.x = int(all_d[0]/rz);
    thisv.y = int(all_d[1]/rz);
    thisv.z = int(all_d[2]/rz);
    all_voxel.insert(thisv);
  }
    octomap_msgs::Octomap octomap_msg;
    octomap_msg.header.stamp = ros::Time::now();
    octomap_msg.header.frame_id = "map";
    octomap_msgs::fullMapToMsg(octomap_map, octomap_msg);


  pcl::VoxelGrid<pcl::PointXYZ> sor;
	sor.setInputCloud(pcl_data);
	sor.setDownsampleAllData(1);
	sor.setLeafSize(0.04, 0.04, 0.04);//设置滤波器处理时采用的体素大小的参数，保留一个体素中所有点的重心
	sor.filter(*pcl_data);//提速非常明显。

  GetArray(mkar);
  file.close();
  sensor_msgs::PointCloud2 pcl_msg;
  pcl::toROSMsg(*pcl_data, pcl_msg);
  pcl_msg.header.frame_id = "map"; // set your desired frame_id
  pcl_msg.height = 1;
  pcl_msg.width = pcl_data->size();
  ros::Rate loop_rate(1);
  while (ros::ok()) {
    pcl_pub.publish(pcl_msg);
    // showp.publish(mkar);
    // octomap_pub.publish(octomap_msg);
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}















 
  