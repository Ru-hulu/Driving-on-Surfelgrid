//自定义点云的情况下,回调函数的执行时间如下.
// 深度转换时间0.055
// 滤波时间0.038
// oct索引时间0.026
// 平面拟合时间0.007
// 总时间0.127
#include "Teclas.hpp"
struct elevation_map
{
  std::vector<float> data;
  float width;
  float height;
  float org_x;
  float org_y;
  float resolution;
};
bool Teclas::SaveMap2txt(std_srvs::SetBool::Request& req,std_srvs::SetBool::Response& res)
{
    float a;
    float b;
    float c;
    float d;
    float typical_z;
    float p_inline; 
    float abs_dis;  
    int   status;   
  std::ofstream file("/home/r/Mysoftware/ExplorationUneven/save_map.txt");
  for(int i=0;i<l_map->width_x;i++)
  {
    for(int j=0;j<l_map->heigh_y;j++)
    {
      Surfel* this_s = l_map->MultiLayer_Grid[i][j];
      if(this_s)
            file << i-l_map->width_x/2 << "," << j-l_map->heigh_y/2 << "\n";
      else continue;
      while(this_s)
      {
            a = this_s->a;
            b = this_s->b;
            c = this_s->c;
            d = this_s->d;
            typical_z = this_s->typical_z;
            p_inline  = this_s->p_inline ;
            abs_dis   = this_s->abs_dis  ;
            status    = this_s->status   ;
            file << std::fixed << std::setprecision(3) << a;
            file << " ";
            file << std::fixed << std::setprecision(3) << b;
            file << " ";
            file << std::fixed << std::setprecision(3) << c;
            file << " ";
            file << std::fixed << std::setprecision(3) << d;
            file << " ";
            file << std::fixed << std::setprecision(3) << typical_z;
            file << " ";
            file << std::fixed << std::setprecision(3) << p_inline;
            file << " ";
            file << std::fixed << std::setprecision(3) << abs_dis;
            file << " ";
            file << status;
            file << " ";
            this_s = this_s->next_level;
      }
      file << "\n";
    }
  } 
  file.close();
  return true;  
}

Teclas::Teclas(const ros::NodeHandle &nh_,std::string static_map_):nh(nh_)
{
  planningspaceanchor_Pub = nh.advertise<geometry_msgs::PoseStamped>("/planning_space_anchor", 1);
  planningspaceanchor_Pub1 = nh.advertise<geometry_msgs::PoseStamped>("/planning_space_anchor1", 1);
  pubCloud = nh.advertise<sensor_msgs::PointCloud2> ("/testpcd", 1);
  // ims=nh.subscribe<sensor_msgs::Image>("/camera/depth/image_raw",5,&Teclas::insertCloudCallback,this);
  ims=nh.subscribe<sensor_msgs::PointCloud2>("/velodyne_points",5,&Teclas::insertCloudCallback,this);

  showp = nh.advertise<visualization_msgs::MarkerArray>("Frontier_Marker",5);
  showMap = nh.advertise<visualization_msgs::MarkerArray>("PlaneMap",5);
  showone = nh.advertise<visualization_msgs::Marker>("onemarker",5);  
  savetxt = nh.advertiseService("save2txt",&Teclas::SaveMap2txt, this);
  // savedebug = nh.advertiseService("savedebug",&Teclas::DebugService,this);  
  showLPS_VP = nh.advertise<visualization_msgs::MarkerArray>("LPS_VP",5);
  showLPS = nh.advertise<visualization_msgs::MarkerArray>("LPSMarker",5);
  showSUBS = nh.advertise<visualization_msgs::MarkerArray>("SUBMarker",5); 
  showcenter = nh.advertise<visualization_msgs::MarkerArray>("Cluster_Center",5); 
  gridmapPub = nh.advertise<nav_msgs::OccupancyGrid>("map2d",5);
  states_client = nh.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
  // map_server = nh.advertiseService("gridmap_server", &Teclas::MapService, this);
  static_map_server = nh.advertiseService("static_map_server", &Teclas::StaticMapService, this);
  static_map = static_map_;
  l_map = new GridSurf(0.25,g_w,g_h);//400 400
}

bool Teclas::compare_center(const std::pair<int, int>& a, const std::pair<int, int>& b) 
{
    int sumA = std::abs(a.first) + std::abs(a.second);
    int sumB = std::abs(b.first) + std::abs(b.second);
    return std::sqrt(sumA) < std::sqrt(sumB);
}


//这里被修改了!!
void Teclas::PublishPlane()
{
  PlaneMarker.markers.clear();
  visualization_msgs::Marker marker;    
  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time::now();
  marker.ns = "array_plane";
  marker.type = visualization_msgs::Marker::CUBE;
  marker.action = visualization_msgs::Marker::ADD;

  marker.color.r = 0.8f;
  marker.color.g = 0.8f;
  marker.color.b = 0.8f;

  marker.color.a = 0.6;
  marker.lifetime = ros::Duration(1.0);/////marker的生存时间只有1s
  marker.scale.x = 0.05;
  marker.scale.y = 0.25;
  marker.scale.z = 0.25;
  float pa=0;  float pb=0;  float pc=0;
  float p1a=0;  float p1b=0;  float p1c=0;
  float ttt;
//向量起点A，就是Pose的position.
//计算将向量[1,0,0]旋转到向量AB方向的四元数。
//target_v = AB/|AB| //单位向量
//r = [1,0,0] X target_v
//r = r/|r| //单位向量
//Theta = arccos(target_v*[1,0,0]) //旋转角
//quaternion = (r*sin(Theta/2),cos(Theta/2))
  float cr1=0;  float cr2=0;
  int attc=0;
  int cct22=0;
  int cct11 = 0;
  int free_cct=0;
  // std::ofstream file_effi("./efficiency.txt",std::ios::app);

  // for(;itt!=(l_map->MultiLayer_Grid).end();itt++)
  float min_z =10000;
  float max_z =-10000;
  std::vector<std::pair<int,int>> all_pair = l_map->exp_grid;
  for(int i=0;i<all_pair.size();i++)
  {
    Surfel* this_s = l_map->find(all_pair[i].first,all_pair[i].second);
    if(this_s==nullptr)continue;//没有用
    while(this_s!=nullptr)
    {   if(this_s->status!=-1)
        {
            min_z = std::min(min_z,this_s->typical_z);  
            max_z = std::max(max_z,this_s->typical_z);
        }
        this_s = this_s->next_level;
    }
  }
  for(int i=0;i<all_pair.size();i++)
  {
    Surfel* this_s = l_map->find(all_pair[i].first,all_pair[i].second);
    if(this_s==nullptr)continue;//没有用
    while(this_s!=nullptr)
    {
        float tem_x; float tem_y;
        l_map->Key2Pos(all_pair[i].first,all_pair[i].second,tem_x,tem_y);
        pa = this_s->a;        pb = this_s->b;        pc = this_s->c;   
                ttt = sqrt(pa*pa+pb*pb+pc*pc);
                pa/=ttt;    pb/=ttt;    pc/=ttt;//target_v
                p1a = 0;    p1b = 0-pc; p1c = pb;//[1 0 0 ]cross pa pb pc
                ttt = sqrt(p1a*p1a+p1b*p1b+p1c*p1c);
                p1a =0; p1b = p1b/ttt; p1c = p1c / ttt;//normalize r
                ttt = pa*1+pb*0+pc*0;
                ttt = std::acos(ttt);//Theta
                geometry_msgs::Quaternion q4;
                q4.x = p1a*sin(ttt/2);    q4.y = p1b*sin(ttt/2);    q4.z = p1c*sin(ttt/2);
                q4.w = cos(ttt/2);
      
        marker.id = attc;
        marker.pose.position.x = tem_x;
        marker.pose.position.y = tem_y;
        marker.pose.position.z = this_s->typical_z;
        marker.pose.orientation.x = q4.x;
        marker.pose.orientation.y = q4.y;
        marker.pose.orientation.z = q4.z;
        marker.pose.orientation.w = q4.w;
        attc++;
        
        if(this_s->status<0)
        {
          marker.scale.x = 0.25;          
          marker.color.r = 1;
          marker.color.g = 0;
          marker.color.b = 0;
          marker.color.a = 1;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;

          // this_s = this_s->next_level;
          // continue;
        }
        else if(this_s->status==0)
        {
          marker.scale.x = 0.05;          
          marker.color.r = 0.5;
          marker.color.g = 0.5;
          marker.color.b = 0.5;
          marker.color.a = 1;
          // this_s = this_s->next_level;
          // continue;
        }
        else if(this_s->status==1)
        {
          float color_range = max_z - min_z;
          float color_value = (this_s->typical_z - min_z) / color_range;

          marker.scale.x = 0.05;
          marker.color.r = std::min(color_value,float(0.8)); // 红色
          marker.color.g = 0; // 绿色
          marker.color.b = 1.0 - color_value; // 蓝色
          marker.color.a = 1;
        }
        if(marker.color.r<=0.81 )
        PlaneMarker.markers.push_back(marker);
        this_s = this_s->next_level;
    }
  }
  showMap.publish(PlaneMarker); 
}
bool Teclas::StaticMapService(std_srvs::SetBool::Request& req,std_srvs::SetBool::Response& res)
{
    std::ifstream read_map(static_map);
    // std::ofstream write_map("/home/r/Mysoftware/ExplorationUneven/map_data/multi_case1_clean.txt");
    // for(int y =-40;y<=-20;y++)
    // {
    //   for(int x = -60;x<=80;x++)
    //   {
    //       std::string this_write_line = "0 0 1 0 0 0 0 1";
    //       write_map<<x<<","<<y<<std::endl;
    //       write_map<<this_write_line<<std::endl;
    //   }
    // }

    int cct = 0;
    // std::ifstream read_map("/home/r/Mysoftware/ExplorationUneven/pointcloud.txt");
    // std::ofstream write_map("/home/r/Mysoftware/ExplorationUneven/map_data/debug_map_fit.txt");
    std::string this_line;
    int k_x;
    int k_y;
    int ccts = 0;
    while(getline(read_map,this_line))
    {
        cct++;
        Surfel* this_s;
        if(this_line.size()<=10)
        {
            int dot_space = this_line.find(',');            
            k_x = stoi(this_line.substr(0,dot_space));
            k_y = stoi(this_line.substr(dot_space+1,this_line.size()-dot_space-1));
            // write_map<<this_line<<std::endl;
        }
        else
        {
            std::istringstream iss(this_line);
            std::string tem_data;
            std::vector<std::string> all_data;
            while(std::getline(iss,tem_data,' '))
            all_data.push_back(tem_data);
            float a,b,c,d,status;

            float ty_z = 0;
            ty_z = std::stof(all_data[4]);
            int flag = 1;

            a = std::stof(all_data[0]);
            b = std::stof(all_data[1]);
            c = std::stof(all_data[2]);
            d = std::stof(all_data[3]);
            status = std::stoi(all_data[7]);
            Surfel* ss = l_map->AddSurf(k_x,k_y,status,a,b,c,d,ty_z,0);
            if(ss!=nullptr)ccts++;
            // std::string this_write_line;
            // if(k_y>=-20 && k_y<=-40 && ty_z>=-0.5)
            // this_write_line = "0 0 1 0 0 0 0 1";
            // else
            // this_write_line = std::to_string(a) + " " + std::to_string(b) +
            // " " + std::to_string(c) + " " + std::to_string(d) + " " + 
            // std::to_string(ty_z) + " " + std::to_string(0) + " " + 
            // std::to_string(0) + " " + std::to_string(flag);
            // write_map<<this_write_line<<std::endl;

            // this_s->a = std::stof(all_data[0]);
            // this_s->b = std::stof(all_data[1]);
            // this_s->c = std::stof(all_data[2]);
            // this_s->d = std::stof(all_data[3]);
            // this_s->typical_z     = std::stof(all_data[4]);
            // this_s->p_inline      = std::stof(all_data[5]);
            // this_s->abs_dis       = std::stof(all_data[6]);
            // this_s->status        = std::stoi(all_data[7]);
            // this_s->next_level    = nullptr;

        }
    }
    ros::Rate rz(1);
    while(ros::ok())
    {
      PublishPlane();
      rz.sleep();      
    }
    return true;
}
void Teclas::Point2SurfelMap(pcl::PointCloud<pcl::PointXYZ>::Ptr in_cloud)
{
                                      //////////把点云栈中的点进行空间分配////////////////////////////////    
                                      std::map<pos_xy,pcl::PointCloud<PointXYZ>::Ptr,CompareKeyGrid> thismap;
                                      int tem_k_x = 0; int tem_k_y = 0;
                                      for(int i =0;i<in_cloud->size();i++)
                                      {
                                            if(l_map->SurfelCheck(in_cloud->points[i].x,in_cloud->points[i].y,in_cloud->points[i].z))
                                            continue;

                                            l_map->Pos2Key(in_cloud->points[i].x,in_cloud->points[i].y,tem_k_x,tem_k_y);
                                            pos_xy thisK(tem_k_x,tem_k_y);

                                            std::map<pos_xy,pcl::PointCloud<PointXYZ>::Ptr,CompareKeyGrid>::iterator thisit = thismap.find(thisK);
                                            if(thisit!=thismap.end())
                                            thisit->second->points.push_back(in_cloud->points[i]);//在当前帧已经标记
                                            else
                                            { //在当前帧中还未标记
                                              pcl::PointCloud<PointXYZ>::Ptr pvector(new pcl::PointCloud<PointXYZ>);
                                              pvector->points.push_back(in_cloud->points[i]);
                                              thismap[thisK]=pvector;
                                            }
                                      }
                                      //////////把点云栈中的点进行空间分配////////////////////////////////    
  double t4=ros::Time::now().toSec();
  // std::cout<<"oct索引时间"<<t4-t2<<std::endl;
  pcl::ModelCoefficients::Ptr plane_model(new pcl::ModelCoefficients);//对点集进行平面拟合操作
  float abs_dis=0;
  float per_inline=0;
  int tra_flag = 0;
  in_cloud->points.clear();
//关于指针https://blog.csdn.net/qq_36403227/article/details/96595707
  //////////这一段进行平面拟合工作，标记了状态是可穿行，不可穿行。////////////////////////////////    
  int cct_cct = 0;
  std::vector<std::pair<pos_xy,Surfel*>> new_surf_pair;
  for(std::map<pos_xy,pcl::PointCloud<PointXYZ>::Ptr,CompareKeyGrid>::iterator it = thismap.begin();it!=thismap.end();it++)
  {
    if(it->second->points.size()<100)
    { //无效的平面标记一下
      continue;
    }
    cct_cct++;
    tra_flag = fit_plane_to_cloud(plane_model,it->second,0.06,per_inline,abs_dis);
    int now_state;
    if(tra_flag<0)//角度很大的平面，内点太少，认为是不可穿行区域
    now_state = -1;
    else if(tra_flag>0)//这里先把frontier和可穿行区域不做区分。0是frontier,1是traversable
    now_state = 1;
    Surfel* new_s = l_map->AddSurf((it->first).x,(it->first).y,now_state,
    plane_model->values[0],plane_model->values[1],plane_model->values[2],plane_model->values[3],
    per_inline,abs_dis);
    if(new_s)
    {
      map_v++;
      new_surf_pair.push_back({it->first,new_s});//把新添加的Surf保存
    }
  }
  std::cout<<"map voxel num is "<<map_v<<std::endl;
  thismap.clear();
  // while(1);
  double t5=ros::Time::now().toSec();
  // std::cout<<"平面拟合时间"<<t5-t4<<std::endl;
  //////////这一段进行平面拟合工作，标记了状态是可穿行，不可穿行。////////////////////////////////    
}
// 正常一帧下，时间开销为
// 深度转换时间0.011
// 滤波时间0.026
// oct索引时间0.007
// 平面拟合时间0
// Frontier动态更新时间0
// 总时间0.045
// 深度信息是50帧，一秒拟合两次，每次用2帧
//建议使用队列结构来处理点云数据
void Teclas::insertCloudCallback(sensor_msgs::PointCloud2 in_cloud)
{
                      Eigen::Matrix4f Twi;  Eigen::Matrix4f Tic;
                      Eigen::Matrix4f sensorToWorld;//这里拿到的就是map->cloud-head变换 
                      gazebo_msgs::GetModelState rq;
                      rq.request.model_name="scout/";
                      rq.request.relative_entity_name="map";
                      while(!states_client.call(rq))
                      std::cout<<"Waiting for gzbo";
                      float posx=rq.response.pose.position.x;
                      float posy=rq.response.pose.position.y;
                      float posz=rq.response.pose.position.z;
                      // std::cout<<"pos "<<posx<<" "<<posy<<" "<<posz<<std::endl;
                                            // geometry_msgs::PoseStamped rbp;
                                            // rbp.header.frame_id = "map";
                                            // rbp.pose.position = rq.response.pose.position;
                                            // rbp.pose.orientation = rq.response.pose.orientation;
                                            // robpos.publish(rbp);
                                            // float yaw_h = tf::getYaw(rq.response.pose.orientation);//这里是车的yaw角
                      tf::Quaternion qw1={rq.response.pose.orientation.x,rq.response.pose.orientation.y,rq.response.pose.orientation.z,rq.response.pose.orientation.w};
                      tf::Matrix3x3 matrixw1;
                      matrixw1.setRotation(qw1);
                      Twi<<matrixw1[0][0],matrixw1[0][1],matrixw1[0][2],posx,
                          matrixw1[1][0],matrixw1[1][1],matrixw1[1][2],posy,
                          matrixw1[2][0],matrixw1[2][1],matrixw1[2][2],posz,
                                  0,0,0,1;
                      Tic<<1,0,0,0.2,
                           0,1,0,0.0,
                           0,0,1,0.2,
                           0,0,0, 1;
                      pcl::PointCloud<pcl::PointXYZ>::Ptr tempcloud(new pcl::PointCloud<pcl::PointXYZ>);
                      pcl::fromROSMsg(in_cloud, *tempcloud);

                          pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
                          // 创建滤波器对象
                          outrem.setInputCloud(tempcloud);
                          outrem.setRadiusSearch(5); // 设置在2m半径的范围内找临近点个数    
                          outrem.setMinNeighborsInRadius(0.6);
                          // 应用滤波器
                          outrem.filter(*tempcloud);  // 把滤波后的点云放回原来的对象

                      pcl::transformPointCloud(*tempcloud, *tempcloud, Twi*Tic);//转到世界坐标系下

                      pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_stack(new pcl::PointCloud<pcl::PointXYZ>);
                      if(cameracct<12)
                      {    
                        point_deque.push_back(tempcloud);
                        cameracct++;
                        // std::cout<<"cameracct "<<cameracct<<std::endl;
                        return;
                      }
                      else if (cameracct==12)
                      {
                        // std::cout<<"12 !! cameracct "<<std::endl;
                        // pcl::PointCloud<pcl::PointXYZ>::Ptr tef = point_deque.front();
                        // tef.reset();
                        point_deque.pop_front();
                        point_deque.push_back(tempcloud);            
                        pcl::PointCloud<pcl::PointXYZ>::Ptr this_pc;
                        for(int i=0;i<=11;i++)
                        {
                          this_pc = point_deque.front();
                          point_deque.pop_front();
                          *pcl_stack += *this_pc;
                          point_deque.push_back(this_pc);
                        }
                      }
  image_in_process = true;
  double t1=ros::Time::now().toSec();  

  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud(pcl_stack);
  pass.setFilterFieldName("z");  // 设置要过滤的维度为z
  pass.setFilterLimits(-3.0, 10.0);  // 设置z坐标的范围限制
  pass.filter(*pcl_stack);  // 将滤波后的点云存储在tempcloud_filtered中

  // std::cout<<"before size is "<<pcl_stack->size()<<std::endl;
  pcl::VoxelGrid<pcl::PointXYZ> sor;
	sor.setInputCloud(pcl_stack);
	sor.setDownsampleAllData(1);
	sor.setLeafSize(0.015, 0.015, 0.015);//设置滤波器处理时采用的体素大小的参数，保留一个体素中所有点的重心
	sor.filter(*pcl_stack);//提速非常明显。
  // std::cout<<"after size is "<<pcl_stack->size()<<std::endl;
                                      // sensor_msgs::PointCloud2 output;
                                      // pcl::toROSMsg(*pcl_stack, output);
                                      // output.header.frame_id = "map";  // 设置合适的frame_id
                                      // pubCloud.publish(output);
  double t2=ros::Time::now().toSec();
  // std::cout<<"time "<<t2-t1<<std::endl;
                                      //////////把点云栈中的点进行空间分配////////////////////////////////    
                                      std::map<pos_xy,pcl::PointCloud<PointXYZ>::Ptr,CompareKeyGrid> thismap;
                                      int tem_k_x = 0; int tem_k_y = 0;
                                      for(int i =0;i<pcl_stack->size();i++)
                                      {
                                            if(l_map->SurfelCheck(pcl_stack->points[i].x,pcl_stack->points[i].y,pcl_stack->points[i].z))
                                            continue;

                                            l_map->Pos2Key(pcl_stack->points[i].x,pcl_stack->points[i].y,tem_k_x,tem_k_y);
                                            pos_xy thisK(tem_k_x,tem_k_y);

                                            std::map<pos_xy,pcl::PointCloud<PointXYZ>::Ptr,CompareKeyGrid>::iterator thisit = thismap.find(thisK);
                                            if(thisit!=thismap.end())
                                            thisit->second->points.push_back(pcl_stack->points[i]);//在当前帧已经标记
                                            else
                                            { //在当前帧中还未标记
                                              pcl::PointCloud<PointXYZ>::Ptr pvector(new pcl::PointCloud<PointXYZ>);
                                              pvector->points.push_back(pcl_stack->points[i]);
                                              thismap[thisK]=pvector;
                                            }
                                      }
                                      //////////把点云栈中的点进行空间分配////////////////////////////////    
  double t4=ros::Time::now().toSec();
  // std::cout<<"oct索引时间"<<t4-t2<<std::endl;
  pcl::ModelCoefficients::Ptr plane_model(new pcl::ModelCoefficients);//对点集进行平面拟合操作
  float abs_dis=0;
  float per_inline=0;
  int tra_flag = 0;
  pcl_stack->points.clear();
//关于指针https://blog.csdn.net/qq_36403227/article/details/96595707
  //////////这一段进行平面拟合工作，标记了状态是可穿行，不可穿行。////////////////////////////////    
  int cct_cct = 0;
  std::vector<std::pair<pos_xy,Surfel*>> new_surf_pair;
  for(std::map<pos_xy,pcl::PointCloud<PointXYZ>::Ptr,CompareKeyGrid>::iterator it = thismap.begin();it!=thismap.end();it++)
  {
    if(it->second->points.size()<100)
    { //无效的平面标记一下
      continue;
    }
    cct_cct++;
    tra_flag = fit_plane_to_cloud(plane_model,it->second,0.06,per_inline,abs_dis);
    int now_state;
    if(tra_flag<0)//角度很大的平面，内点太少，认为是不可穿行区域
    now_state = -1;
    else if(tra_flag>0)//这里先把frontier和可穿行区域不做区分。0是frontier,1是traversable
    now_state = 1;
    Surfel* new_s = l_map->AddSurf((it->first).x,(it->first).y,now_state,
    plane_model->values[0],plane_model->values[1],plane_model->values[2],plane_model->values[3],
    per_inline,abs_dis);
    if(new_s)
    {
      map_v++;
      new_surf_pair.push_back({it->first,new_s});//把新添加的Surf保存
    }
  }
  std::cout<<"map voxel num is "<<map_v<<std::endl;
  thismap.clear();
  // while(1);
  double t5=ros::Time::now().toSec();
  // std::cout<<"平面拟合时间"<<t5-t4<<std::endl;
  //////////这一段进行平面拟合工作，标记了状态是可穿行，不可穿行。////////////////////////////////    

  // std::cout<<"The size of new voxel is "<<cct_cct<<std::endl;

  std::vector<std::pair<pos_xy,Surfel*>> newFrontier_Set;  
  std::vector<std::pair<pos_xy,Surfel*>> chenode_set;//存放活跃区已经是frontier的节点

  float w_x = 0 ;
  float w_y = 0 ;
  float w_z = 0 ;
  int im_fronter=0;//判定当前的it是否为新的frontier  
    //新node可穿行,那么就可能是Frontier,因为当前帧的TRA FRON不区分
                      //1、拿到周围的所有点，并且将frotier放入check_list。
                      //2、如果周围所有点中出现了一个未知点，自己加入new_list
    //新node不可穿行
                      //1、拿到当前node周围的6个点，并且将frotier从标记为trav并且删掉。
  for(int i=0;i<new_surf_pair.size();i++)
  {
    if(new_surf_pair[i].second->status==Teclas::TRAVE)
    { 
      //可以通行表示有可能是frontier
      im_fronter = 0;
      for(int kkx = -1;kkx<=1;kkx++)
      {
        for(int kky = -1;kky<=1;kky++)
        {
          if(kkx == 0 && kky==0)continue;          
          l_map->Key2Pos(new_surf_pair[i].first.x + kkx,new_surf_pair[i].first.y + kky,w_x,w_y);//邻居的世界坐标
          w_z = new_surf_pair[i].second->typical_z;
          Surfel* nei_sur = l_map->SurfelCheck(w_x,w_y,w_z);//检查邻居是否存在
          if(nei_sur==nullptr && im_fronter==0)im_fronter = 1;//邻居而且周围没有危险
          else if(nei_sur!=nullptr)
          {
            if(nei_sur->status==Teclas::UNTRA)im_fronter = -1;//周围有危险
            else if(nei_sur->status==Teclas::FRONT)
            chenode_set.push_back({pos_xy(new_surf_pair[i].first.x + kkx,new_surf_pair[i].first.y + kky),nei_sur});//周围是front需要检查
          }
        }
      }
      if(im_fronter==1) newFrontier_Set.push_back({new_surf_pair[i].first,new_surf_pair[i].second});
    }
    else
    {
      for(int kkx = -1;kkx<=1;kkx++)
      {
        for(int kky = -1;kky<=1;kky++)
        {
          if(kkx == 0 && kky==0)continue;          
          l_map->Key2Pos(new_surf_pair[i].first.x + kkx,new_surf_pair[i].first.y + kky,w_x,w_y);//邻居的世界坐标
          w_z = new_surf_pair[i].second->typical_z;
          Surfel* nei_sur = l_map->SurfelCheck(w_x,w_y,w_z);//检查邻居是否存在
          if(nei_sur==nullptr)continue;
          else if(nei_sur->status==Teclas::FRONT)
          {
            //delete it from frontier_set;
            nei_sur->status = Teclas::TRAVE;
            std::map<pos_xy,std::vector<float>,CompareKeyGrid>::iterator st_itt = 
            frontier.find(pos_xy(new_surf_pair[i].first.x + kkx,new_surf_pair[i].first.y + kky));
            int f_ct = 0;
            for(f_ct = 0;f_ct<st_itt->second.size();f_ct++)
            {
              if(std::abs(st_itt->second[f_ct]-w_z)<=10 * resolu)
              break;
            }
            st_itt->second.erase(st_itt->second.begin()+f_ct);
            if(st_itt->second.size()==0)frontier.erase(st_itt);
            //delete it from frontier_set;
          }
        }
      }
    }
  }

  //这里有可能有重复元素！！
  for(int i=0;i<chenode_set.size();i++)
  {
      im_fronter = 0;
      for(int kkx = -1;kkx<=1;kkx++)
      {
        for(int kky = -1;kky<=1;kky++)
        {
          if(kkx == 0 && kky==0)continue;          
          l_map->Key2Pos(chenode_set[i].first.x + kkx,chenode_set[i].first.y + kky,w_x,w_y);
          w_z = chenode_set[i].second->typical_z;
          Surfel* nei_sur = l_map->SurfelCheck(w_x,w_y,w_z);
          if(nei_sur==nullptr)im_fronter = 1;//周围有未知而且周围没有危险
          else if(nei_sur->status==Teclas::UNTRA)
          {
            im_fronter = -1;//周围有危险
            break;
          }
        }
      }
      if(im_fronter==1)continue;
      else if(im_fronter==0 || im_fronter==-1)
      {
            chenode_set[i].second->status = Teclas::TRAVE;//可通行
            std::map<pos_xy,std::vector<float>,CompareKeyGrid>::iterator st_itt = 
            frontier.find(pos_xy(chenode_set[i].first.x, chenode_set[i].first.y));
            if(st_itt==frontier.end())continue;
            int f_ct = 0;
            for(f_ct = 0;f_ct<st_itt->second.size();f_ct++)
            {
              if(std::abs(st_itt->second[f_ct]-w_z)<=10 * resolu)
              break;
            }
            if(f_ct<st_itt->second.size())//这里有bug，但是先不管
            {
              st_itt->second.erase(st_itt->second.begin()+f_ct);
              if(st_itt->second.size()==0)
              frontier.erase(st_itt);
            }
      }
  }

        // std::cout<<"4444444444444"<<std::endl;
  chenode_set.clear();
  for(int i=0;i<newFrontier_Set.size();i++)
  {
    newFrontier_Set[i].second->status = Teclas::FRONT;
    std::map<pos_xy, std::vector<float>, CompareKeyGrid>::iterator this_itt =  frontier.find(newFrontier_Set[i].first);
    if(this_itt==frontier.end())
    frontier[newFrontier_Set[i].first] = {newFrontier_Set[i].second->typical_z};
    else    
    frontier[newFrontier_Set[i].first].push_back(newFrontier_Set[i].second->typical_z);
  }
  newFrontier_Set.clear();
        // std::cout<<"5555555555555555555555"<<std::endl;
  double t6=ros::Time::now().toSec();
 
  double t7=ros::Time::now().toSec();
  std::cout<<"The size of frontiers is "<<frontier.size()<<std::endl;
  //std::cout<<"子地图更新时间"<<t7-t6<<std::endl;
  // std::cout<<"地图更新总时间"<<t7-t1<<std::endl;
  int d1=0;
  PublishPlane();
  // PublishFrontierMarker();
  // LPS_Planner();
  // image_in_process = false;
  // ForCheck();
}



//返回<0,表示平面不可穿行，返回>=0，表示散点到平面的距离绝对值之和。
int Teclas::fit_plane_to_cloud(pcl::ModelCoefficients::Ptr coefficients,const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,const double dist_thes,float& precent_inline,float& varia)
{
  std::cout<<cloud->points.size()<<std::endl;
  float sqd = 0;
  float all_dis=0;
  // std::cout<<"fit_plane_to_cloud"<<std::endl;
  try 
  {
  // std::cout<<"fit_plane_to_cloud1"<<std::endl;
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::SACSegmentation<pcl::PointXYZ> seg;//平面拟合
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(dist_thes);
    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);
    float a=coefficients->values[0];
    float b=coefficients->values[1];
    float c=coefficients->values[2];
    float d=coefficients->values[3];
    sqd=std::sqrt(a*a+b*b+c*c);
  // std::cout<<"fit_plane_to_cloud2"<<std::endl;
    float xita = acos(c/sqd);//法向量与垂直z的夹角，相当平面与水平面的夹角。
    precent_inline = inliers->indices.size()/cloud->points.size();
    if(xita>xita_thred||(precent_inline<0.7))
    {
      precent_inline = 0;
      for(int i=0;i<cloud->points.size();i++)
      precent_inline += cloud->points[i].z;
      precent_inline /= float(cloud->points.size());
      return -1;//内点不够多，角度太大，放弃
    } 
    all_dis=0;
    for(int i=0;i<cloud->points.size();i++)
    {
      all_dis+=std::abs(a*cloud->points[i].x+
                        b*cloud->points[i].y+
                        c*cloud->points[i].z+
                        d);
    }
  // std::cout<<"fit_plane_to_cloud3"<<std::endl;
  all_dis = all_dis/(cloud->points.size());
  varia=all_dis/sqd;
  // std::cout<<"varia "<<varia<<"precent_inline "<<precent_inline<<std::endl;
  return 1;
  }
  catch (...) 
  {
    coefficients->values.push_back(0.0);
    coefficients->values.push_back(0.0);
    coefficients->values.push_back(0.0);
    coefficients->values.push_back(0.0);
    return -1;
  }
}


int main (int argc, char** argv)
{
  ros::init(argc,argv,"octomap_server");
  ros::NodeHandle nh;
  std::string static_map_path;
  nh.getParam("static_map_path", static_map_path);
  Teclas T(nh,static_map_path);
  while(ros::ok())
  {
    ros::spinOnce();
  }
  return (0);
}