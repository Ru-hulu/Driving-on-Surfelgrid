#include"algorithm.hpp"
#include"CorridorBuilder2d.hpp"
using namespace HybridAStar;
void Algorithm::VisualPoly(float x1,float y1,float z1,
float x2,float y2,float z2,
float w_x,float w_y,float w_z,
float color_r,float color_g,float color_b)
{
    visualization_msgs::Marker line1;
    line1.header.frame_id = "map";
    line1.header.stamp = ros::Time::now();
    line1.lifetime = ros::Duration(1.0);/////marker的生存时间只有1s
    line1.ns = "all_poly";    
    line1.type   = visualization_msgs::Marker::LINE_STRIP;
    line1.action = visualization_msgs::Marker::ADD;
    line1.scale.x = 0.03; // 线的宽度

    line1.id = cctt;
    cctt++;
    // 添加直线的点
    geometry_msgs::Point point1, point2;
    point1.x = x1;
    point1.y = y1;
    point1.z = z1;
    line1.points.push_back(point1);

    point2.x = x2;
    point2.y = y2;
    point2.z = z2;
    line1.points.push_back(point2);
        line1.color.r = color_r;
        line1.color.g = color_g;
        line1.color.b = color_b;
        line1.color.a = 1.0;
    allMarker.markers.push_back(line1);


    visualization_msgs::Marker center_point;
    center_point.header.frame_id = "map";
    center_point.header.stamp = ros::Time::now();
    center_point.lifetime = ros::Duration(1.0);/////marker的生存时间只有1s
    center_point.ns = "all_poly";    
    center_point.type   = visualization_msgs::Marker::CUBE;
    center_point.action = visualization_msgs::Marker::ADD;
    center_point.scale.x = 0.05; // 线的宽度    
    center_point.scale.y = 0.05; // 线的宽度    
    center_point.scale.z = 0.05; // 线的宽度    
    center_point.color.r = color_r;
    center_point.color.g = color_g;
    center_point.color.b = color_b;
    center_point.color.a = 1.0;
    center_point.pose.position.x = w_x;
    center_point.pose.position.y = w_y;
    center_point.pose.position.z = w_z;
    center_point.id = cctt;
    cctt++;
    allMarker.markers.push_back(center_point);
}
void Cost_obs(Eigen::Vector2f point,Eigen::Vector3f constaint_parm,float th,float& loss,Eigen::Vector2f& grad_this)
{   //传入的都是p坐标系下的坐标和约束
    float this_dis = constaint_parm(0)*point(0) + constaint_parm(1)*point(1) + constaint_parm(2);
    // std::cout<<"dis is "<<this_dis<<std::endl;
    if(this_dis>=th)
    {
        loss = 0.0;
        grad_this = {0,0};
    }
    else
    {
        grad_this = {constaint_parm(0),constaint_parm(1)};
        loss = th - this_dis;
        loss = std::min(loss/th,float(1.0));           
        grad_this = grad_this * loss;
    }
}
void Obstacle_Term(const Eigen::Vector4f& car_center,const Eigen::Matrix4f& Twp,
const std::vector<Eigen::Vector3f>local_constraint,const float& obs_safty_th, float& loss,Eigen::Vector3f& grad_this)
{
    loss = 0.0;
    grad_this={0.0,0.0,0.0};
    for(int c = 0;c<local_constraint.size();c++)
    {
        float tem_loss = 0.0;
        Eigen::Vector2f local_grad_x_y;
        Eigen::Vector4f local_grad;
        Eigen::Vector4f local_org={0,0,0,1};
        Eigen::Vector4f word_grad;
        Cost_obs({car_center(0),car_center(1)},local_constraint[c],obs_safty_th,tem_loss,local_grad_x_y);
        loss += tem_loss;
        local_grad(0) = local_grad_x_y(0);
        local_grad(1) = local_grad_x_y(1);
        local_grad(2) = 0;//在局部坐标系下,梯度的表示
        local_grad(3) = 1;
        // if(std::abs(local_grad(0))>=1e-3||std::abs(local_grad(1))>=1e-3)
        // std::cout<<car_center(0)<<" "<<car_center(1)<<" "<<
        // local_grad(0)<<" "<<local_grad(1)<<std::endl;
        word_grad = Twp * (local_grad-local_org);
        grad_this(0)    += word_grad(0)*(-1.0);
        grad_this(1)    += word_grad(1)*(-1.0);
        grad_this(2)    += word_grad(2)*(-1.0);   
    }   //将梯度转换到世界坐标系下
    //障碍物项
}
void Smooth_Term(Eigen::Vector3f pr2,Eigen::Vector3f pr1,Eigen::Vector3f p,
Eigen::Vector3f nx1,Eigen::Vector3f nx2,float a,float b,float c,float d,float& loss,Eigen::Vector3f& grad_this)
{
        loss = 0;
        grad_this = {0.0,0.0,0.0};
        float const_v = a * pr2(0) + b * pr2(1) + c * pr2(2) + d;
        float const_norm = sqrt(a*a+b*b+c*c);
        pr2(0) = pr2(0) - const_v*a/const_norm;
        pr2(1) = pr2(1) - const_v*b/const_norm;
        pr2(2) = pr2(2) - const_v*c/const_norm;

        const_v = a * pr1(0) + b * pr1(1) + c * pr1(2) + d;
        pr1(0) = pr1(0) - const_v*a/const_norm;
        pr1(1) = pr1(1) - const_v*b/const_norm;
        pr1(2) = pr1(2) - const_v*c/const_norm;

        const_v = a * nx1(0) + b * nx1(1) + c * nx1(2) + d;
        nx1(0) = nx1(0) - const_v*a/const_norm;
        nx1(1) = nx1(1) - const_v*b/const_norm;
        nx1(2) = nx1(2) - const_v*c/const_norm;

        const_v = a * nx2(0) + b * nx2(1) + c * nx2(2) + d;
        nx2(0) = nx2(0) - const_v*a/const_norm;
        nx2(1) = nx2(1) - const_v*b/const_norm;
        nx2(2) = nx2(2) - const_v*c/const_norm;
        //需要投影的平面
        grad_this += 2.0*(p+nx2-2.0*nx1);
        grad_this += 4.0*(2.0*p-pr1-nx1); 
        grad_this += 2.0*(pr2+p-2.0*pr1);

        loss += std::sqrt(
                            (nx1[0]+pr1[0]-2*p[0])*(nx1[0]+pr1[0]-2*p[0])+
                            (nx1[1]+pr1[1]-2*p[1])*(nx1[1]+pr1[1]-2*p[1])+
                            (nx1[2]+pr1[2]-2*p[2])*(nx1[2]+pr1[2]-2*p[2])
                         );
}

void Feasibility_Term(const std::vector<double>& x_,float& loss,vector<double>&grad) 
{
    double ts, vm2, am2, ts_inv2, ts_inv4;
    vm2 = Constants::max_vel * Constants::max_vel;
    am2 = Constants::max_acc * Constants::max_acc;
    ts      = Constants::ts;
    ts_inv2 = 1 / ts / ts;
    ts_inv4 = ts_inv2 * ts_inv2;
    loss = 0.0;

    int point_num = x_.size()/3;
    for (int i = 0; i < point_num - 1; i++) 
    {
        Eigen::Vector3d vi(x_[(i+1)*3  ]-x_[(i)*3  ], 
                         x_[(i+1)*3+1]-x_[(i)*3+1], 
                         x_[(i+1)*3+2]-x_[(i)*3+2]);
      for (int j = 0; j < 3; j++) 
      {
        double vd = vi(j) * vi(j) * ts_inv2 - vm2;
        if (vd > 0.0) 
        {
          loss += pow(vd, 2);
          double temp_v = 4.0 * vd * ts_inv2;
          grad[(i+0)*3+j] += -temp_v * vi(j);
          grad[(i+1)*3+j] += temp_v * vi(j);
        }//这里只会对一个维度比如x\y\z进行修改
      }
    }
    for (int i = 0; i < point_num - 2; i++)
    {
        Eigen::Vector3d ai(x_[(i+2)*3  ]+x_[(i)*3  ]-2*x_[(i+1)*3  ], 
                           x_[(i+2)*3+1]+x_[(i)*3+1]-2*x_[(i+1)*3+1], 
                           x_[(i+2)*3+2]+x_[(i)*3+2]-2*x_[(i+1)*3+2]);
        for (int j = 0; j < 3; j++) 
        {
            double ad = ai(j) * ai(j) * ts_inv4 - am2;
            if (ad > 0.0) 
            {
                loss += pow(ad, 2);
                double temp_a = 4.0 * ad * ts_inv4;
                grad[(i+0)*3+j] += temp_a * ai(j);
                grad[(i+1)*3+j] += -2 * temp_a * ai(j);
                grad[(i+2)*3+j] += temp_a * ai(j);
            }
        }
    }
}

void Algorithm::UNE_opt()
{
    UNE_Prepare_Obs();
    UNE_Opt_Traj();
    UNE_Time_Adjust();//到这里，所有的优化，时间分配都已经结束了
    // int point_num = une_tra_data_opt.size()/3;
    // for (int i = 0; i < point_num - 1; i++) 
    // {
    //     Eigen::Vector3d vi(une_tra_data_opt[i+1][0]-une_tra_data_opt[i][0], 
    //                        une_tra_data_opt[i+1][1]-une_tra_data_opt[i][1], 
    //                        une_tra_data_opt[i+1][2]-une_tra_data_opt[i][2]);
    //     vi = vi/Constants::ts;
    //     if(vi.norm()>=Constants::max_vel)
    //     std::cout<<"vi is "<<vi(0)<<" "<<vi(1)<<" "<<vi(2)<<" "<<std::endl;
    // }

    // for (int i = 0; i < point_num - 2; i++)
    // {
    //     Eigen::Vector3d ai(une_tra_data_opt[i+2][0]+une_tra_data_opt[i][0]-2*une_tra_data_opt[i+1][0], 
    //                        une_tra_data_opt[i+2][1]+une_tra_data_opt[i][1]-2*une_tra_data_opt[i+1][1], 
    //                        une_tra_data_opt[i+2][2]+une_tra_data_opt[i][2]-2*une_tra_data_opt[i+1][2]);
    //     ai = ai/Constants::ts/Constants::ts;
    //     if(ai.norm()>=Constants::max_acc)
    //     std::cout<<"ai is "<<ai(0)<<" "<<ai(1)<<" "<<ai(2)<<" "<<std::endl;
    // }

}
void Algorithm::UNE_Prepare_Obs()
{
    double prepare_time1 = ros::Time::now().toSec();
    Eigen::Matrix4f Twp = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f Tpw = Eigen::Matrix4f::Identity();
    une_tra_planes_Twp.push_back(Twp);//开头的平面坐标系我们不关心
    une_tra_planes_Tpw.push_back(Tpw);//开头的平面坐标系我们不关心

    nei_obs_list.resize(une_tra_data.size());//头部不需要障碍物扩展
    allMarker.markers.clear();
    for(int i=1;i<une_tra_data.size()-1;i++)//尾部也不需要障碍物扩展
    {
        float pre_node[3] = {une_tra_data[i-1][0],une_tra_data[i-1][1],une_tra_data[i-1][2]};
        float now_node[3] = {une_tra_data[i  ][0],une_tra_data[i  ][1],une_tra_data[i  ][2]};
        float nex_node[3] = {une_tra_data[i+1][0],une_tra_data[i+1][1],une_tra_data[i+1][2]};
        float v_p[2] = {pre_node[0]-now_node[0],pre_node[1]-now_node[1]};
        float v_n[2] = {nex_node[0]-now_node[0],nex_node[1]-now_node[1]};
        float l = std::sqrt(v_p[0]*v_p[0]+v_p[1]*v_p[1]);
        v_p[0] = v_p[0]/l;
        v_p[1] = v_p[1]/l;
        l = std::sqrt(v_n[0]*v_n[0]+v_n[1]*v_n[1]);
        v_n[0] = v_n[0]/l;
        v_n[1] = v_n[1]/l;
        float v_ext[2] = {v_n[0]+v_p[0],v_n[1]+v_p[1]};
        l = std::sqrt(v_ext[0]*v_ext[0]+v_ext[1]*v_ext[1]);
        if(l<=1e-3)
        {
            v_ext[0] = 0-v_n[1];
            v_ext[1] = v_n[0];
        }//两条线几乎平行，直接取垂直
        else
        {
            v_ext[0] /= l;
            v_ext[1] /= l;
        }
        int step_ext = int(obs_ext_dis/obs_rs);

        //正方向扩展
        for(int ii=1;ii<=step_ext;ii++)
        {
            int useless_z = 0;
            float nei_x = now_node[0]+obs_rs*v_ext[0]*float(ii);
            float nei_y = now_node[1]+obs_rs*v_ext[1]*float(ii);            
            Surfel* this_sur = SearchinMap(nei_x,nei_y,now_node[2],useless_z);
            if(this_sur==nullptr)
            {
                Eigen::Vector3f this_obs;
                this_obs(0) = nei_x;
                this_obs(1) = nei_y;
                this_obs(2) = now_node[2];
                nei_obs_list[i].first = this_obs;
                break;                
            }
            else if(this_sur->status==HybridAStar::UNTRA)
            {
                Eigen::Vector3f this_obs;
                this_obs(0) = nei_x;
                this_obs(1) = nei_y;
                this_obs(2) = this_sur->typical_z;
                nei_obs_list[i].first = this_obs;
                break;                
            }
            //对周围的环境进行扩展
            if(ii==step_ext)
            {
                Eigen::Vector3f this_obs;
                this_obs(0) = nei_x;
                this_obs(1) = nei_y;
                this_obs(2) = this_sur->typical_z;
                nei_obs_list[i].first = this_obs;
            }//补充一个边界
        }
        //正方向扩展

        //反方向扩展
        for(int ii=1;ii<=step_ext;ii++)
        {
            int useless_z = 0;
            float nei_x = now_node[0]+obs_rs*v_ext[0]*float(ii)*(-1.0);
            float nei_y = now_node[1]+obs_rs*v_ext[1]*float(ii)*(-1.0);            
            Surfel* this_sur = SearchinMap(nei_x,nei_y,now_node[2],useless_z);
            if(this_sur==nullptr)
            {
                Eigen::Vector3f this_obs;
                this_obs(0) = nei_x;
                this_obs(1) = nei_y;
                this_obs(2) = now_node[2];
                nei_obs_list[i].second = this_obs;
                break;                
            }
            else if(this_sur->status==HybridAStar::UNTRA)
            {
                Eigen::Vector3f this_obs;
                this_obs(0) = nei_x;
                this_obs(1) = nei_y;
                this_obs(2) = this_sur->typical_z;
                nei_obs_list[i].second = this_obs;
                break;                
            }
            //对周围的环境进行扩展
            if(ii==step_ext)
            {
                Eigen::Vector3f this_obs;
                this_obs(0) = nei_x;
                this_obs(1) = nei_y;
                this_obs(2) = this_sur->typical_z;
                nei_obs_list[i].second = this_obs;
            }//补充一个边界
        }
        //反方向扩展
        if(i>=20)
        {
            // VisualMark(nei_obs_list[i].first(0),nei_obs_list[i].first(1),nei_obs_list[i].first(2),0,0);
            // VisualMark(nei_obs_list[i].second(0),nei_obs_list[i].second(1),nei_obs_list[i].second(2),0,0);
        }
        int useless_z = 0;
        Surfel* now_plane = SearchinMap(now_node[0],now_node[1],now_node[2],useless_z);
        float axis_x[3] = {0,0,0};
        float axis_y[3] = {0,0,0};
        float axis_z[3] = {now_plane->a,now_plane->b,now_plane->c};        
        float dirc_v[3] = {now_node[0],now_node[1],now_node[2]};
        axis_x[0] = dirc_v[0] + 1.0;
        axis_x[1] = dirc_v[1];
        axis_x[2] = dirc_v[2];
        float const_v = now_plane->a * axis_x[0] + now_plane->b * axis_x[1] + now_plane->c * axis_x[2] + now_plane->d;
        axis_x[0] = axis_x[0] - const_v * now_plane->a;
        axis_x[1] = axis_x[1] - const_v * now_plane->b;
        axis_x[2] = axis_x[2] - const_v * now_plane->c;

        axis_x[0] -= dirc_v[0];
        axis_x[1] -= dirc_v[1];
        axis_x[2] -= dirc_v[2];//平面x轴的方向

        float length_x = sqrt(axis_x[0]*axis_x[0]+axis_x[1]*axis_x[1]+axis_x[2]*axis_x[2]);
        axis_x[0] /= length_x;
        axis_x[1] /= length_x;
        axis_x[2] /= length_x;//平面x轴的归一化
        if(axis_z[2]<0)
        {
            axis_z[0] = 0.0-axis_z[0];
            axis_z[1] = 0.0-axis_z[1];
            axis_z[2] = 0.0-axis_z[2];
        }

        axis_y[0] = axis_z[1] * axis_x[2] - axis_z[2] * axis_x[1];
        axis_y[1] = axis_z[2] * axis_x[0] - axis_z[0] * axis_x[2];
        axis_y[2] = axis_z[0] * axis_x[1] - axis_z[1] * axis_x[0];
        Twp<<axis_x[0],axis_y[0],axis_z[0],dirc_v[0],        
                    axis_x[1],axis_y[1],axis_z[1],dirc_v[1],
                    axis_x[2],axis_y[2],axis_z[2],dirc_v[2],
                    0        ,        0,        0,1;
        une_tra_planes_Twp.push_back(Twp);//Twp
        
        Tpw.topLeftCorner(3,3) = Twp.topLeftCorner(3,3).transpose();
        Tpw.topRightCorner(3,1) = -1.0 * Tpw.topLeftCorner(3,3) * Twp.topRightCorner(3,1);
        une_tra_planes_Tpw.push_back(Tpw);//Tpw
    }
    // while(1)
    // tp_b.publish(allMarker);
    Twp = Eigen::Matrix4f::Identity();
    une_tra_planes_Twp.push_back(Twp);//末尾的平面坐标系我们不关心

    Tpw = Eigen::Matrix4f::Identity();
    une_tra_planes_Tpw.push_back(Tpw);//末尾的平面坐标系我们不关心

    une_tra_constraint.resize(une_tra_planes_Twp.size());
    //现在每一个位置左右两侧最近的障碍物都已经准备好了。
    allMarker.markers.clear();
    // std::vector<std::vector<float>> resize_tra_data;
    // std::vector<Eigen::Matrix4f> resize_Twp;
    // std::vector<Eigen::Matrix4f> resize_Tpw;
    // std::vector<std::vector<Eigen::Vector3f>> resize_constraint;
    std::ofstream fout("/home/r/Mysoftware/ExplorationUneven/map_data/uselesss.txt");
    std::cout<<"now show poly !!!!!!!!!!!!!!!!!!!!!!!!"<<std::endl;            
    for(int i=4;i<nei_obs_list.size()-5;i++)
    {
        Tpw = une_tra_planes_Tpw[i];
        Twp = une_tra_planes_Twp[i];
                                // resize_tra_data.push_back(une_tra_data[i]);
                                // resize_Tpw.push_back(Tpw);
                                // resize_Twp.push_back(Twp);
        std::vector<Eigen::Vector2f> obs_xoy_pro;
        //世界坐标系下，采样点周围的障碍物在采样点坐标系xoy平面上的投影
        for(int n = -3;n<=3;n++)
        {//对于采样点前后的若干个采样点
            Eigen::Vector4f this_obs;
            this_obs.topRows(3) = nei_obs_list[i+n].first;
            this_obs(3) = 1.0;
            this_obs = Tpw * this_obs;
            obs_xoy_pro.push_back({this_obs(0),this_obs(1)});//障碍物在采样点坐标系下的表示,仅考虑xoy平面
            
            this_obs.topRows(3) = nei_obs_list[i+n].second;
            this_obs(3) = 1.0;
            this_obs = Tpw * this_obs;
            obs_xoy_pro.push_back({this_obs(0),this_obs(1)});//障碍物在采样点坐标系下的表示,仅考虑xoy平面
        }
        std::vector<cv::Point2f>final_vertex;
        std::vector<Eigen::Vector3f>constraints;
        corridorBuilder2d(0,0,3.0,obs_xoy_pro,final_vertex,constraints);        
        une_tra_constraint[i] = constraints;
                                // resize_constraint.push_back(constraints);
        //建立凸包约束
    double prepare_time2 = ros::Time::now().toSec();
    prepare_time = prepare_time2-prepare_time1;



        srand((unsigned int)time(NULL));//srand为随机种子挑选代码;()内为随机种子; 
        float colors[3] = {0.0,0.0,0.0};
        colors[i%3] = 1.0;
        for(int f=0;f<final_vertex.size();f++)
        {
            int n_f = (f+1)%final_vertex.size();
            Eigen::Vector4f p1,p2;
            p1(0) = final_vertex[f].x;               p2(0) = final_vertex[n_f].x;                        
            p1(1) = final_vertex[f].y;               p2(1) = final_vertex[n_f].y;                        
            p1(2) = 0;                               p2(2) = 0;            
            p1(3) = 1;                               p2(3) = 1;            
            p1 = Twp * p1;
            p2 = Twp * p2;
            VisualPoly(p1(0),p1(1),p1(2),p2(0),p2(1),p2(2),
            Twp(0,3),Twp(1,3),Twp(2,3),
            colors[0],colors[1],colors[2]);
             fout<<std::to_string(p1(0))<<" "<<std::to_string(p1(1))<<" "<<std::to_string(p1(2))
            <<" "<<std::to_string(p2(0))<<" "<<std::to_string(p2(1))<<" "<<std::to_string(p2(2))<<std::endl;
        }
    }
    std::cout<<"now show poly !!!!!!!!!!!!!!!!!!!!!!!!"<<std::endl;            
    // while(1)
    // tp_b.publish(allMarker);
    

    // une_tra_data = resize_tra_data;
    // une_tra_planes_Tpw = resize_Tpw;
    // une_tra_planes_Twp = resize_Twp;
    // une_tra_constraint = resize_constraint;
}


void Algorithm::Uev_GetStepPath(Node3D* start_node)
{
    Node3D* now_node = start_node;
    std::vector<Node3D*> all_node;
    
    while(now_node!=nullptr)
    {   //这里的轨迹是倒序的
        all_node.push_back(now_node);
        // std::cout<<now_node->float_x<<" "<<now_node->float_y<<" "<<now_node->float_z<<
        // " "<<now_node->t<<std::endl;
        now_node = now_node->parent;
        // VisualMark(now_node->float_x,now_node->float_y,now_node->float_z,now_node->t,0);
    }
    if(une_tra_data.size()>0)all_node.pop_back();
    step_path_cct++;
    must_node.push_back(step_path_cct);
    une_tra_data.push_back({all_node.back()->float_x,
    all_node.back()->float_y,all_node.back()->float_z});
    for(int i=all_node.size()-2;i>=1;i--)
    {   
        step_path_cct++;
        if((all_node[i]->prim<=2 && all_node[i+1]->prim>=3)||
           (all_node[i]->prim>=3 && all_node[i+1]->prim<=2))
        must_node.push_back(step_path_cct);
        une_tra_data.push_back({all_node[i]->float_x,
        all_node[i]->float_y,all_node[i]->float_z});

    }
    step_path_cct++;
    must_node.push_back(step_path_cct);
}

void Algorithm::UNE_Time_Adjust()
{
    Eigen::MatrixXd ctrl_pts;
    ctrl_pts.resize(une_tra_data_opt.size(),3);
    for(int i=0;i<une_tra_data_opt.size();i++)
    {
        ctrl_pts.row(i) = Eigen::Vector3d(une_tra_data_opt[i][0],une_tra_data_opt[i][1],une_tra_data_opt[i][2]);
    }
    double ts = Constants::dubinsStepSize/Constants::max_vel;
    NonUniformBspline pos = NonUniformBspline(ctrl_pts, 3, ts);
    pos.setPhysicalLimits(Constants::max_vel, Constants::max_acc);
    bool feasible = pos.checkFeasibility(false);
    int iter_num = 0;
    while (!feasible && ros::ok()) 
    {
      feasible = pos.reallocateTime();
      if (++iter_num >= 5) break;
    }
    
    pos.getMaxCurvature(path_max_curvature);
    double m_;
    pos.getMeanAndMaxAcc(m_,path_max_acc);
    double            tm, tmp;
    pos.getTimeSpan(tm, tmp);

    Eigen::Vector3d pre_pre(une_tra_data_opt[0][0],une_tra_data_opt[0][1],une_tra_data_opt[0][2]);
    Eigen::Vector3d     pre(une_tra_data_opt[1][0],une_tra_data_opt[1][1],une_tra_data_opt[1][2]);
    path_smooth_cost = 0;
    for(int i=2;i<une_tra_data_opt.size();i++)
    {
        Eigen::Vector3d     nnow(une_tra_data_opt[i][0],une_tra_data_opt[i][1],une_tra_data_opt[i][2]);
        path_smooth_cost += ((nnow-pre) - (pre-pre_pre)).norm();
        pre_pre = pre;
        pre = nnow;
    }
    path_len_opt = 0;
    for (double t = tm+0.01; t <= tmp; t += 0.01) 
    {
        Eigen::VectorXd pred = pos.evaluateDeBoor(t-0.01);
        Eigen::VectorXd nowd = pos.evaluateDeBoor(t);
        path_len_opt += (nowd-pred).norm();
    }

    for (double t = tm; t <= tmp; t += 0.01) 
    {
        Eigen::VectorXd nowd = pos.evaluateDeBoor(t);
        Eigen::Vector3d n(nowd(0),nowd(1),nowd(2));
        spline_result.push_back(n);
    }
}
void Algorithm::UNE_Opt_Traj()
{
    double optt1 = ros::Time::now().toSec();
    Bspline_Fit();//把路径点拟合到b样条曲线上，现在une_tra_data已经是控制点
    double optt2 = ros::Time::now().toSec();

    for(int i=0;i<une_tra_data.size();i++)
    {
        int useless_z = 0;
        Surfel* this_patch = SearchinMap(une_tra_data[i][0],une_tra_data[i][1],une_tra_data[i][2],useless_z);
        path_local_geometry.push_back(this_patch);
    }
    double optt3 = ros::Time::now().toSec();
    nlopt::opt opt_ct(nlopt::algorithm(11),une_tra_data.size()*3);//优化器初始化 11 的效果还可以1.6
    opt_ct.set_min_objective(Algorithm::costFunction,this); //参数：优化函数，在优化过程中需要的变量
    opt_ct.set_maxeval(500);//优化次数
    opt_ct.set_maxtime(une_opt_iteration);
    opt_ct.set_xtol_rel(1e-5);//当dx / x 小于该数值，优化停止

    std::vector<double> initial_x;//被优化的控制点
    std::vector<double> lb_x;//优化下边界
    std::vector<double> ub_x;//优化上边界
    for(int i=0;i<une_tra_data.size();i++)
    {
        initial_x.push_back(une_tra_data[i][0]);
        initial_x.push_back(une_tra_data[i][1]);
        initial_x.push_back(une_tra_data[i][2]);
        lb_x.push_back(une_tra_data[i][0]-opt_bound);
        lb_x.push_back(une_tra_data[i][1]-opt_bound);
        lb_x.push_back(une_tra_data[i][2]-opt_bound/2.0);
        ub_x.push_back(une_tra_data[i][0]+opt_bound);
        ub_x.push_back(une_tra_data[i][1]+opt_bound);
        ub_x.push_back(une_tra_data[i][2]+opt_bound/2.0);
    }
    opt_ct.set_lower_bounds(lb_x);
    opt_ct.set_upper_bounds(ub_x);
    bool success_flag = true;
    try
    {
        double cost_r=0;
        nlopt::result this_result = opt_ct.optimize(initial_x,cost_r);
        std::cout<<"Finish optimization "<<cost_r<<std::endl;
    }
    catch(std::exception& e)
    {
        std::cout<<"nlopt exception: "<<e.what()<<std::endl;
        success_flag = false;
    }
    double optt4 = ros::Time::now().toSec();
    opt_path_search_time = optt4 - optt3 + optt2 - optt1;

    if(success_flag)
    {
        une_tra_data_opt.resize(une_tra_data.size());
        for(int i=0;i<une_tra_data.size();i++)
        {
            une_tra_data_opt[i].push_back(initial_x[3*i]  );
            une_tra_data_opt[i].push_back(initial_x[3*i+1]);            
            une_tra_data_opt[i].push_back(Caculate_z(initial_x[3*i],initial_x[3*i+1],initial_x[3*i+2]).second);
        }
    }
}
double Algorithm::costFunction(const std::vector<double> &x, std::vector<double>& grad ,void* func_data)
{
    Eigen::Vector3f p_sub2{0,0,0};//当前节点前后节点在当前节点局部平面上的投影 在世界坐标下的表示
    Eigen::Vector3f p_sub1{0,0,0};//当前节点前后节点在当前节点局部平面上的投影 在世界坐标下的表示
    Eigen::Vector3f p     {0,0,0};
    Eigen::Vector3f p_add1{0,0,0};//当前节点前后节点在当前节点局部平面上的投影 在世界坐标下的表示
    Eigen::Vector3f p_add2{0,0,0};//当前节点前后节点在当前节点局部平面上的投影 在世界坐标下的表示
    int x_index = 0;//被优化参数的索引
    Algorithm* obj_self = reinterpret_cast<Algorithm*>(func_data);
    int point_number = obj_self->une_tra_data.size();//轨迹长度
    float car_size_half_x = obj_self->car_size_x_2;
    float car_size_half_y = obj_self->car_size_y_2;
    std::vector<std::vector<float>> tem_path  = obj_self->une_tra_data;
    x_index = 6;
    int now_point_index = x_index/3;
    double loss_this_iteration = 0.0;
    obj_self->opt_cct++;
    float weight_smooth = obj_self->une_smo_loss_w;
    float weight_obstacle = obj_self->une_obs_loss_w;
    float weight_feasibility = obj_self->une_fea_loss_w;
    for(int i=0;i<grad.size();i++)grad[i] = 0.0;
    while(x_index<=(x.size()-9))
    {
        now_point_index = x_index/3;

        p_sub2 = {float(x[x_index-6]),float(x[x_index-5]),float(x[x_index-4])};
        p_sub1 = {float(x[x_index-3]),float(x[x_index-2]),float(x[x_index-1])};
        p      = {float(x[x_index]  ),float(x[x_index+1]),float(x[x_index+2])};
        p_add1 = {float(x[x_index+3]),float(x[x_index+4]),float(x[x_index+5])};
        p_add2 = {float(x[x_index+6]),float(x[x_index+7]),float(x[x_index+8])};

        //平滑项
        //需要投影的平面
        float pro_a,pro_b,pro_c,pro_d;
        pro_a = obj_self->path_local_geometry[now_point_index]->a;
        pro_b = obj_self->path_local_geometry[now_point_index]->b;
        pro_c = obj_self->path_local_geometry[now_point_index]->c;
        pro_d = obj_self->path_local_geometry[now_point_index]->d;
        //需要投影的平面
        float smooth_loss_t = 0.0;
        Eigen::Vector3f smooth_loss_grad;
        Smooth_Term(p_sub2,p_sub1,p,p_add1,p_add2,pro_a,pro_b,pro_c,pro_d,smooth_loss_t,smooth_loss_grad);
        grad[x_index]   += smooth_loss_grad(0)*weight_smooth;
        grad[x_index+1] += smooth_loss_grad(1)*weight_smooth;
        grad[x_index+2] += smooth_loss_grad(2)*weight_smooth;
        loss_this_iteration += smooth_loss_t*weight_smooth;
        //平滑项

        // Eigen::Vector3f dir1 = {p_index_add1[0] - p_index[0],p_index_add1[1] - p_index[1],p_index_add1[2]-p_index[2]};
        // Eigen::Vector3f dir2 = {p_index[0] - p_index_sub1[0],p_index[1] - p_index_sub1[1],p_index[2]-p_index_sub1[2]};
        // dir1.normalized();dir2.normalized();
        // Eigen::Vector3f head_cor_dir = dir1+dir2;
        //这是投影的车头方向,世界坐标
        // head_cor_dir.normalized();
        // Eigen::Vector3f this_z_axis = {pro_a,pro_b,pro_c};
        // Eigen::Vector3f head_ver_dir = head_cor_dir.cross(this_z_axis);
        //局部平面坐标系下,当前车的yaw角度被cor_dir确定了
        //然后对凸包约束进行处理
        // std::vector<Eigen::Vector3f> car_corner = {
        // {p_index[0] + head_cor_dir(0)*car_size_half_x   +  head_cor_dir(0)*car_size_half_y,
        //  p_index[1] + head_cor_dir(1)*car_size_half_x   +  head_cor_dir(1)*car_size_half_y,
        //  p_index[2] + head_cor_dir(2)*car_size_half_x   +  head_cor_dir(2)*car_size_half_y},
        // {p_index[0] + head_cor_dir(0)*car_size_half_x   -  head_cor_dir(0)*car_size_half_y,
        //  p_index[1] + head_cor_dir(1)*car_size_half_x   -  head_cor_dir(1)*car_size_half_y,
        //  p_index[2] + head_cor_dir(2)*car_size_half_x   -  head_cor_dir(2)*car_size_half_y},
        // {p_index[0] - head_cor_dir(0)*car_size_half_x   +  head_cor_dir(0)*car_size_half_y,
        //  p_index[1] - head_cor_dir(1)*car_size_half_x   +  head_cor_dir(1)*car_size_half_y,
        //  p_index[2] - head_cor_dir(2)*car_size_half_x   +  head_cor_dir(2)*car_size_half_y},
        // {p_index[0] - head_cor_dir(0)*car_size_half_x   -  head_cor_dir(0)*car_size_half_y,
        //  p_index[1] - head_cor_dir(1)*car_size_half_x   -  head_cor_dir(1)*car_size_half_y,
        //  p_index[2] - head_cor_dir(2)*car_size_half_x   -  head_cor_dir(2)*car_size_half_y}
        // };//世界坐标系下的车身坐标,但是满足在局部拟合平面上
        // std::vector<Eigen::Vector3f> car_corner = {{p_index[0],p_index[1],p_index[2]}};
         //世界坐标系下的车身坐标,但是满足在局部拟合平面上


        //障碍物项
        float obs_loss_t = 0.0;
        Eigen::Vector3f obs_loss_grad;
        Eigen::Vector4f car_center = {p(0),p(1),p(2),1};
        std::vector<Eigen::Vector3f> local_constraint = obj_self->une_tra_constraint[now_point_index];
        Eigen::Matrix4f Twp = obj_self->une_tra_planes_Twp[now_point_index];
        Eigen::Matrix4f Tpw = obj_self->une_tra_planes_Tpw[now_point_index];
        car_center = Tpw*car_center;//已经被转移到局部坐标系下了
        Obstacle_Term(car_center,Twp,local_constraint,obj_self->obs_safty_th,obs_loss_t,obs_loss_grad);
        loss_this_iteration += obs_loss_t*weight_obstacle;
        grad[x_index]   += obs_loss_grad(0)*weight_obstacle;
        grad[x_index+1] += obs_loss_grad(1)*weight_obstacle;
        grad[x_index+2] += obs_loss_grad(2)*weight_obstacle;   
        //障碍物项

        // std::cout<<"car "<<x_index/3<<" grad "
        // <<grad[x_index]  <<" "
        // <<grad[x_index+1]<<" "
        // <<grad[x_index+2]<<" "<<std::endl;
        x_index+=3;
    }

    //加速度和速度项
    float fea_cost=0;
    std::vector<double> fea_grad(grad.size(),0);
    Feasibility_Term(x,fea_cost,fea_grad);
    loss_this_iteration += fea_cost*weight_feasibility;
    for(int i=0;i<grad.size();i++)
    grad[i] += fea_grad[i]*weight_feasibility;
    //加速度和速度项

    grad[0] = 0.0; grad[1] = 0.0; grad[2] = 0.0;//头不动
    grad[3] = 0.0; grad[4] = 0.0; grad[5] = 0.0;//头不动

    grad[3*point_number-1] = 0.0; grad[3*point_number-2] = 0.0; grad[3*point_number-3] = 0.0;//尾不动
    grad[3*point_number-4] = 0.0; grad[3*point_number-5] = 0.0; grad[3*point_number-6] = 0.0;//尾不动
    // std::cout<<"opt_cct: "<<obj_self->opt_cct<<" loss_this_iteration "<<loss_this_iteration<<std::endl;

    if(loss_this_iteration<=obj_self->min_loss)
    {
        obj_self->une_tra_data_opt.clear();
        obj_self->min_loss = loss_this_iteration;
        obj_self->une_tra_data_opt.resize(obj_self->une_tra_data.size());
        for(int i=0;i<obj_self->une_tra_data.size();i++)
        {
            obj_self->une_tra_data_opt[i].push_back(x[3*i]  );
            obj_self->une_tra_data_opt[i].push_back(x[3*i+1]);
            obj_self->une_tra_data_opt[i].push_back(x[3*i+2]);
        }
    }

    return loss_this_iteration;
}

void Algorithm::Bspline_Fit()
{
    NonUniformBspline bspline;
    std::vector<Eigen::Vector3d>tra_way_point;
    std::vector<Eigen::Vector3d>start_end_ve_acc;
    start_end_ve_acc.push_back(Eigen::Vector3d(0,0,0));//开始时的速度
    start_end_ve_acc.push_back(Eigen::Vector3d(0,0,0));//开始时的加速度
    start_end_ve_acc.push_back(Eigen::Vector3d(0,0,0));//结束时的速度
    start_end_ve_acc.push_back(Eigen::Vector3d(0,0,0));//结束时的加速度
    Eigen::MatrixXd ctrl_pts;
    for(int i=0;i<une_tra_data.size();i++)
    {
        Eigen::Vector3d this_wp(une_tra_data[i][0],une_tra_data[i][1],une_tra_data[i][2]);
        tra_way_point.push_back(this_wp);
    }
    float ts = Constants::dubinsStepSize/max_max_velocity;  
    bspline.parameterizeToBspline(ts,tra_way_point,start_end_ve_acc,ctrl_pts);
    une_tra_data.clear();
    for(int i=0;i<ctrl_pts.rows();i++)
    {
        une_tra_data.push_back({float(ctrl_pts(i,0)),float(ctrl_pts(i,1)),float(ctrl_pts(i,2))});
    }
    //疑问：为什么需要用B样条对原始轨迹点先进行拟合，然后再进行优化？
    //回答：因为原始轨迹不能计算导数，而B样条轨迹可以计算导数。这样就可以对速度，加速度进行约束和优化
}