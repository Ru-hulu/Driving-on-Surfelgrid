//大框架
//1、计算Astar距离得到阈值
//2、采用Hybrid采样开始计算
//3、采样给定补偿计算阈值
//   3.1如果在阈值范围内直接采用投影ReedShep
//   3.2如果在阈值范围外回到2
#include"nav_uneven.hpp"

Uneven_Navigator::Uneven_Navigator(const ros::NodeHandle &nh_,std::string map_string_)
{
    nh = nh_;
    navserver = nh.advertiseService("uneven_nav",&Uneven_Navigator::NavService,this);
    org_pathpub = nh.advertise<nav_msgs::Path>("org_path",5); 
    opt_pathpub = nh.advertise<nav_msgs::Path>("opt_path",5); 
    searchpub = nh.advertise<visualization_msgs::MarkerArray>("search_pub",5);
    increpub = nh.advertise<visualization_msgs::MarkerArray>("incre_pub",5);
    MileStonepub = nh.advertise<visualization_msgs::MarkerArray>("milstone_pub",5);
    map_string = map_string_;
    width_x=1000;//400
    heigh_y=1000;//400
}
void Uneven_Navigator::Initstartend(float sxw,float syw,float szw,float syaw,
        float txw,float tyw,float tzw,float tyaw)
{
    start_x = sxw;
    start_y = syw;
    start_z = szw;
    start_yaw = syaw;

    target_x = txw;
    target_y = tyw;
    target_z = tzw;
    target_yaw = tyaw;
}
bool Uneven_Navigator::NavService(std_srvs::SetBool::Request& req,std_srvs::SetBool::Response& res)
{
    Surfel*** MultiLayer_Grid = InitialMap();
    float map_res=0.25;

    // start_x  = 3.5*0.25;     start_y  = 4.5*0.25; start_z = ((-0.139)*start_x + (0.096)*start_y - 0.646) / (0-0.986); start_yaw = 0;
    // target_x = 12.5*0.25;    target_y = 3.5*0.25; target_z = (0.046*target_x + 0.064*target_y - 0.985)  / (0-0.997); target_yaw = -1.57;

    // start_x  = 3.5*0.25;     start_y  = 4.5*0.25; start_z = ((-0.139)*start_x + (0.096)*start_y - 0.646) / (0-0.986); start_yaw = 0;
    // target_x = 12.5*0.25;    target_y = 10.5*0.25; target_z = (0.102*target_x + 0.013*target_y - 1.068)  / (0-0.995); target_yaw = 0;
    //这组数据有点问题，没有按照最短路径进行扩展

    // start_x  = 3.5*0.25;     start_y  = 4.5*0.25; start_z = ((-0.139)*start_x + (0.096)*start_y - 0.646) / (0-0.986); start_yaw = 0;
    // target_x = 14.5*0.25;    target_y = 10.5*0.25; target_z = (0.102*target_x + 0.013*target_y - 1.068)  / (0-0.995); target_yaw = 0;

    // start_x  = 4.5*0.25;     start_y  = 3.5*0.25; start_z = ((-0.086)*start_x + (0.046)*start_y - 0.654) / (0-0.995); start_yaw = 0;
    // target_x = 6.5*0.25;    target_y = 12.5*0.25; target_z = ((-0.079)*start_x + (-0.003)*start_y - 0.537) / (0-0.997); target_yaw = 3.14159;
    
    //检验组1： 长距离最远倒车测试 debug_map
    // start_x  = 12.5*0.25; start_y  = 10.5*0.25; start_yaw = 3.14159;    // // 212 210
    // start_z = ((0.071)*start_x + (0.046)*start_y - 0.983) / (0-0.996);    
    // target_x = 20.5*0.25; target_y = -20.5*0.25; target_yaw = 0;        // // 220 179
    // target_z = ((0.011)*target_x + (-0.099)*target_y + 0.095) / (0-0.995);
    //检验组1： 长距离最远倒车测试
    
    //检验组1： 直线测试
    // start_x  = 3.5*0.25;     start_y  = 0; start_z = ((-0.139)*start_x + (0.096)*start_y - 0.646) / (0-0.986); start_yaw = 0;
    // target_x = 13.5*0.25;    target_y = 0; target_z = ((-0.139)*target_x + (0.096)*target_y - 0.646) / (0-0.986); target_yaw = 0;
    //检验组1： 直线测试

    // 0.011 -0.099 0.995 0.095 -0.662 1.000 0.002 1 
    //txt文档中的200,200是原点
    HybridAStar::Algorithm alg_handle(width_x,heigh_y,map_res,MultiLayer_Grid,searchpub,increpub);

    double time1 = ros::Time::now().toSec();//从182 188 开始是dubin曲线
    vector<vector<float>> all_milestones;
    alg_handle.PrepareNavi(start_x, start_y, start_z, start_yaw, 
    target_x, target_y, target_z, target_yaw, all_milestones);
    // for(int i=0;i<all_milestones.size();i++)
    // {
        // std::cout<<all_milestones[i][0]<<" "
        // <<all_milestones[i][1]<<" "
        // <<all_milestones[i][2]<<" "
        // <<all_milestones[i][3]<<std::endl;
    // }
    Eigen::Vector4f step_start={start_x, start_y, start_z, start_yaw};
    for(int i=0;i<all_milestones.size();i++)
    {
        Node3D* last_node = alg_handle.hybridAStar(
        step_start(0), step_start(1), 
        step_start(2), step_start(3), 
        all_milestones[i][0], all_milestones[i][1], 
        all_milestones[i][2], all_milestones[i][3]);     

        if(last_node)
        alg_handle.Uev_GetStepPath(last_node);
        else
        {
            std::cout<<"no solution"<<std::endl;
        }
        step_start = {
        last_node->float_x,
        last_node->float_y,
        last_node->float_z,
        last_node->t};   
        alg_handle.ClearFrontDataSet();
    }

    double time2 = ros::Time::now().toSec();
    std::cout<<time1<<" "<<time2<<std::endl;
    org_path_search_time = time2-time1;
    std::cout<<"take time "<<org_path_search_time<<std::endl;
    //做一些可視化。
    origin_path.poses.clear();
    origin_path.header.frame_id = "map";
    une_opt_path.poses.clear();
    une_opt_path.header.frame_id = "map";
    alg_handle.UNE_opt();

    std::ofstream output_file(result_path);
    if(output_file.is_open())
    for(auto this_pos:alg_handle.spline_result)
    output_file<<this_pos(0)<<" "<<this_pos(1)<<" "<<this_pos(2)<<std::endl; 

output_file << "-----------------------------------------" << std::endl;
output_file << "Time cost of the initial trajectory: " << org_path_search_time << std::endl;
output_file << "Time cost of the trajectory optimization: " << alg_handle.opt_path_search_time + alg_handle.prepare_time << std::endl;
output_file << "Trajectory length: " << alg_handle.path_len_opt << std::endl;
output_file << "Smoothing loss: " << alg_handle.path_smooth_cost << std::endl;
output_file << "Maximum acceleration: " << alg_handle.path_max_acc << std::endl;
output_file << "Maximum curvature: " << alg_handle.path_max_curvature << std::endl;

std::cout << "The algorithm indicators of Driving on pointcloud are as follows:" << std::endl;
std::cout << "Time cost of the initial trajectory: " << org_path_search_time << std::endl;
std::cout << "Time cost of the trajectory optimization: " << alg_handle.opt_path_search_time + alg_handle.prepare_time << std::endl;
std::cout << "Trajectory length: " << alg_handle.path_len_opt << std::endl;
std::cout << "Smoothing loss: " << alg_handle.path_smooth_cost << std::endl;
std::cout << "Maximum acceleration: " << alg_handle.path_max_acc << std::endl;
std::cout << "Maximum curvature: " << alg_handle.path_max_curvature << std::endl;


    double time3 = ros::Time::now().toSec();
    std::cout<<"Time cost 2 "<<time3-time2<<std::endl;
    // alg_handle.Driving_on_PCL_opt(last_node);
    for(int i=0;i<alg_handle.une_tra_data.size();i++)
    {
        geometry_msgs::PoseStamped thisp;
        thisp.header.frame_id = "map";
        thisp.pose.position.x = alg_handle.une_tra_data[i][0];
        thisp.pose.position.y = alg_handle.une_tra_data[i][1];
        thisp.pose.position.z = alg_handle.une_tra_data[i][2];
        origin_path.poses.push_back(thisp);
    }
    std::cout<<"--------------------------"<<std::endl;
    // for(int i=0;i<alg_handle.une_tra_data_opt.size();i++)
    for(int i=0;i<alg_handle.spline_result.size();i++)
    {
        geometry_msgs::PoseStamped thisp;
        thisp.header.frame_id = "map";
        thisp.pose.position.x = alg_handle.spline_result[i](0);
        thisp.pose.position.y = alg_handle.spline_result[i](1);
        thisp.pose.position.z = alg_handle.spline_result[i](2);
        une_opt_path.poses.push_back(thisp);
    }
    vMarker_arr.markers.clear();
    for(int i=0;i<all_milestones.size();i++)
    VisualMark(all_milestones[i][0],all_milestones[i][1],
    all_milestones[i][2],all_milestones[i][3]);

    // std::ofstream output_file("/home/r/Mysoftware/ExplorationUneven/path_data/traj.txt",std::ios_base::app);
    // output_file<<map_string<<std::endl;
    // for(int i=0;i<alg_handle.une_tra_data_opt.size();i++)
    // {
    //     output_file<<alg_handle.une_tra_data_opt[i][0]<<" "
    //     <<alg_handle.une_tra_data_opt[i][1]<<" "
    //     <<alg_handle.une_tra_data_opt[i][2]<<std::endl;
    // }   
    ros::Rate loop_rate(10);
    while(1)
    {
        MileStonepub.publish(vMarker_arr);
        // std::cout<<"time is "<< time2 - time1 <<std::endl;
        ShowTraj();
        alg_handle.tp_b.publish(alg_handle.allMarker);
        loop_rate.sleep();
        // searchpub.publish(alg_handle.allMarker);
    }
    for(std::pair<int,int> t_p:valid_key)
    {
        Surfel* this_f =  MultiLayer_Grid[t_p.first][t_p.second];
        Surfel* next_f;
        while (this_f)
        {
            next_f = this_f->next_level;
            delete this_f;
            this_f = next_f;
        }
    }
    //这里是指针变量 内存的释放
    for(int i=0;i<width_x;i++)
    delete[] MultiLayer_Grid[i];
    delete[] MultiLayer_Grid;
    return true;
}



void Uneven_Navigator::VisualMark(float w_x,float w_y, float w_z,float yaw)
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time::now();
  marker.ns = "unev_vm";
  marker.type = visualization_msgs::Marker::ARROW;

  marker.color.r = 0.8f;
  marker.color.g = 0.8f;
  marker.color.b = 0.8f;
  vmcct++;
  marker.id = vmcct;
  marker.color.a = 1.0;
  marker.lifetime = ros::Duration(1.0);/////marker的生存时间只有1s
  marker.scale.x = 0.25;
  marker.scale.y = 0.02;
  marker.scale.z = 0.02;

  marker.pose.position.x = w_x;
  marker.pose.position.y = w_y;
  marker.pose.position.z = w_z;

  geometry_msgs::Quaternion q;//初始化四元数（geometry_msgs类型）
  q=tf::createQuaternionMsgFromRollPitchYaw(0,0,yaw);//欧拉角转四元数（geometry_msgs::Quaternion）
  marker.pose.orientation = q;
  vMarker_arr.markers.push_back(marker);
}

void Uneven_Navigator::ShowTraj()
{
    org_pathpub.publish(origin_path);
    opt_pathpub.publish(une_opt_path);
}

Surfel*** Uneven_Navigator::InitialMap()
{
    std::ifstream read_map;
    read_map.open(map_string);

    Surfel*** MultiLayer_Grid = new Surfel**[width_x];//400
    //MultiLayer_Grid是一个指针数组 
    for(int i=0;i<width_x;i++)//400
    MultiLayer_Grid[i] = new Surfel*[heigh_y];//400
    //MultiLayer_Grid[i] 也是一个指针数组
    std::string this_line;
    int k_x_w;
    int k_y_w;
    int k_x;
    int k_y;
    int line_cct = 0;
    while(getline(read_map,this_line))
    {
        Surfel* this_s;
        if(this_line.size()<=10)
        {
            //以后地图文件里统一放k_x_w
            //以后地图文件里统一放k_y_w
            int dot_space = this_line.find(',');            
            k_x_w = stoi(this_line.substr(0,dot_space));
            k_y_w = stoi(this_line.substr(dot_space+1,this_line.size()-dot_space-1));
            k_x = k_x_w + width_x/2;
            k_y = k_y_w + heigh_y/2; 
            if(MultiLayer_Grid[k_x][k_y]==nullptr)
            {
                MultiLayer_Grid[k_x][k_y] = new Surfel();                 
                this_s = MultiLayer_Grid[k_x][k_y];
                valid_key.push_back({k_x,k_y});
            }
            else
            {
                Surfel* tem_s = MultiLayer_Grid[k_x][k_y];
                while(tem_s->next_level)tem_s = tem_s->next_level;
                tem_s->next_level = new Surfel();
                this_s = tem_s->next_level;
            }
        }
        else
        {
            std::istringstream iss(this_line);
            std::string tem_data;
            std::vector<std::string> all_data;
            while(std::getline(iss,tem_data,' '))
            all_data.push_back(tem_data);
            this_s->a = std::stof(all_data[0]);
            this_s->b = std::stof(all_data[1]);
            this_s->c = std::stof(all_data[2]);
            this_s->d = std::stof(all_data[3]);
            this_s->typical_z     = std::stof(all_data[4]);
            this_s->p_inline      = std::stof(all_data[5]);
            this_s->abs_dis       = std::stof(all_data[6]);
            this_s->status        = std::stoi(all_data[7]);
            this_s->next_level    = nullptr;
        }
    }
    valid_key.shrink_to_fit();
    return MultiLayer_Grid;
}