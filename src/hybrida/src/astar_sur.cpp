#include "astar_sur.hpp"
#include<limits.h>
 
AstarSurPlanner::AstarSurPlanner(Surfel*** MultiLayer_Grid_,int width_x_,int heigh_y_,float rz_,ros::Publisher & in_pub)
{
    MultiLayer_Grid  =  MultiLayer_Grid_;
    map_width_x = width_x_;
    map_heigh_y = heigh_y_;
    rz = rz_;                                                    
    test_pub = in_pub;
}

//这个函数是用来计算启发函数的，返回值以grid为单位
float AstarSurPlanner::AstarPlan(float start_x, float start_y, float start_z, float target_x, float target_y, float target_z)
{

                                                    // visualization_msgs::Marker marker;
                                                    // marker.header.frame_id = "map";
                                                    // marker.header.stamp = ros::Time::now();
                                                    // marker.ns = "now_search";
                                                    // marker.type = visualization_msgs::Marker::CUBE;
                                                    // marker.action = visualization_msgs::Marker::ADD;

                                                    // marker.color.r = 0.8f;
                                                    // marker.color.g = 0.8f;
                                                    // marker.color.b = 0.8f;
                                                    // int cctt = 0;
                                                    // marker.color.a = 1.0;
                                                    // marker.lifetime = ros::Duration(1.0);/////marker的生存时间只有1s
                                                    // marker.scale.x = 0.2;
                                                    // marker.scale.y = 0.2;
                                                    // marker.scale.z = 0.2;

    int start_key_x = int(start_x/rz);
    int start_key_y = int(start_y/rz);
    int start_key_z = 0;
    if(start_key_x<0) start_key_x--;
    if(start_key_y<0) start_key_y--;
    start_key_x+=map_width_x/2;
    start_key_y+=map_heigh_y/2;

    int target_key_x = int(target_x/rz);
    int target_key_y = int(target_y/rz);
    int target_key_z = 0;
    if(target_key_x<0) target_key_x--;
    if(target_key_y<0) target_key_y--;
    target_key_x+=map_width_x/2;
    target_key_y+=map_heigh_y/2;

    std::set<Surf3d*,Comparexyz> find_cost;//所有的节点，包括开和关
    std::priority_queue<Surf3d*,std::vector<Surf3d*>,Comparecost> open_check;//所有的开节点
    Surf3d* currentNode(new Surf3d());

    currentNode->key_x = start_key_x;
    currentNode->key_y = start_key_y;
    int kz = -1;
    Surfel* cc =  MultiLayer_Grid[start_key_x][start_key_y];
    while(std::abs(cc->typical_z-start_z)>1.5*rz){cc = cc->next_level;kz++;}
    currentNode->key_z = kz;

    // currentNode->h =(std::abs(currentNode->key_x - target_key_x)+
    //                      std::abs(currentNode->key_y - target_key_y)+
    //                      std::abs(currentNode->key_z - target_key_z));
    currentNode->h =10 * std::abs(currentNode->key_x - target_key_x)+
                    10 * std::abs(currentNode->key_y - target_key_y);
    currentNode->g = 0;
    currentNode->cost = currentNode->g + currentNode->h;
    currentNode->is_open = true;
    currentNode->parent = nullptr;
    currentNode->fz = start_z;
    open_check.push(currentNode);
    find_cost.insert(currentNode);

        double time1 = ros::Time::now().toSec();
        bool search_flag = false;
        float target_fz = target_z; 
        while(!open_check.empty())
        {
            currentNode = open_check.top();
                                        //Uncommenting this section to view the search process.
                                        // cctt++;
                                        // marker.id = cctt;
                                        // marker.pose.position.x = rz * float(currentNode->key_x-map_width_x/2);
                                        // marker.pose.position.y = rz * float(currentNode->key_y-map_heigh_y/2);
                                        // marker.pose.position.z = currentNode->fz;
                                        // allMarker.markers.push_back(marker);
                                        //Uncommenting this section to view the search process.
            open_check.pop();
            currentNode->is_open = false;//注意，只会从open_check中删除，但是不会从find_cost中删除！！！！！
            if (
                (target_key_x == currentNode->key_x) && 
                (target_key_y == currentNode->key_y) && 
                (std::abs(target_z - currentNode->fz)<=2.5*rz)
               )
            {
                search_flag = true;
                break;
            }

            for(int i=0;i<xy_patch.size();i++)
            {
                int nei_x = xy_patch[i].first  + currentNode->key_x;
                int nei_y = xy_patch[i].second + currentNode->key_y;//邻居key

                Surfel* this_nei_node = MultiLayer_Grid[nei_x][nei_y];//邻居
                Surfel* pre_node = this_nei_node;
                int cct_z = 0;
                while(this_nei_node)
                {
                    if(std::abs(this_nei_node->typical_z - currentNode->fz)<=1.5*rz)
                    {
                                        Surf3d* temp_neibour(new Surf3d());                    
                                        temp_neibour->key_x = nei_x;
                                        temp_neibour->key_y = nei_y;
                                        temp_neibour->key_z = cct_z;
                                        if(find_cost.find(temp_neibour)!=find_cost.end())
                                        {
                                            delete temp_neibour;
                                            break;
                                        }
                                        else
                                        {//这已经说明不在close和open中了
                                            temp_neibour->parent = currentNode;
                                            temp_neibour->h =  10*std::abs(temp_neibour->key_x-target_key_x)+
                                                               10*std::abs(temp_neibour->key_y-target_key_y);
                                            temp_neibour->g = currentNode->g + cost_move[i];
                                            temp_neibour->cost = temp_neibour->g + temp_neibour->h;
                                            temp_neibour->is_open = true;
                                            temp_neibour->fz = this_nei_node->typical_z;
                                            open_check.push(temp_neibour);
                                            find_cost.insert(temp_neibour);
                                            break;
                                        }
                    } 
                    else
                    {
                        pre_node = this_nei_node;
                        this_nei_node = this_nei_node->next_level;
                        cct_z++;
                    }
                } 
            }
        }
                                //Uncommenting this section to view the search process.
                                //Surf3d* t_node = currentNode;
                                //while(t_node!=nullptr)
                                //{
                                // cctt++;
                                // marker.id = cctt;
                                // marker.pose.position.x = rz * float(t_node->key_x-map_width_x/2);
                                // marker.pose.position.y = rz * float(t_node->key_y-map_heigh_y/2);
                                // marker.pose.position.z = t_node->fz;
                                // allMarker.markers.push_back(marker);
                                // t_node = t_node->parent;
                                //}
                                // ros::Rate raz(1);
                                // while(ros::ok())
                                // {
                                //     raz.sleep();
                                //     test_pub.publish(allMarker);
                                // }
                                //Uncommenting this section to view the search process.            

        float step_cost = 0.0;
        if(search_flag) 
        {
        //   geometry_msgs::PoseStamped pose_this;
        //   pose_this.header.stamp = ros::Time::now();
        //   pose_this.header.frame_id = "map";  
        //   pose_this.pose.orientation.x = 0.0;
        //   pose_this.pose.orientation.y = 0.0;
        //   pose_this.pose.orientation.z = 0.0;
        //   pose_this.pose.orientation.w = 1.0;
        //   path_range_min_x = INT32_MAX;
        //   path_range_min_y = INT32_MAX;
        //   path_range_max_x = INT32_MIN;
        //   path_range_max_y = INT32_MIN;
          Surf3d* preNode = currentNode;
          currentNode = currentNode->parent;
          while(currentNode)
          {
            float dx = currentNode->key_x - preNode->key_x;
            float dy = currentNode->key_y - preNode->key_y;
            float dz = currentNode->fz - preNode->fz;
            dx *= rz;
            dy *= rz;
            step_cost += std::sqrt(dx*dx+dy*dy);
            // plan_key.push_back({currentNode->key_x,currentNode->key_y});
            // path_range_min_x = std::min(path_range_min_x,currentNode->key_x);
            // path_range_min_y = std::min(path_range_min_y,currentNode->key_y);
            // path_range_max_x = std::max(path_range_max_x,currentNode->key_x);
            // path_range_max_y = std::max(path_range_max_y,currentNode->key_y);
            preNode = currentNode;
            currentNode = currentNode->parent;
          }
        }
        for(std::set<Surf3d*,Comparexyz>::iterator it = find_cost.begin();it!=find_cost.end();it++)
        delete *it;
        if(search_flag)return step_cost;
        else return -1;
}


//这个函数是用来计算路径中间点的,要求整条路径满足安全阈值
int AstarSurPlanner::GetMilestones(float start_x, float start_y, float start_z, float start_yaw, 
float target_x, float target_y, float target_z, float target_yaw, 
std::vector<std::vector<float>>& milestone)
{
                                                //Uncommenting this section to view the search process.
                                                    // visualization_msgs::Marker marker;
                                                    // marker.header.frame_id = "map";
                                                    // marker.header.stamp = ros::Time::now();
                                                    // marker.ns = "now_search";
                                                    // marker.type = visualization_msgs::Marker::CUBE;
                                                    // marker.action = visualization_msgs::Marker::ADD;

                                                    // marker.color.r = 0.8f;
                                                    // marker.color.g = 0.0f;
                                                    // marker.color.b = 0.0f;
                                                    // int cctt = 0;
                                                    // marker.color.a = 1.0;
                                                    // marker.lifetime = ros::Duration(1.0);/////marker的生存时间只有1s
                                                    // marker.scale.x = 0.2;
                                                    // marker.scale.y = 0.2;
                                                    // marker.scale.z = 0.2;

                                                    // cctt++;
                                                    // marker.id = cctt;
                                                    // marker.pose.position.x = start_x;
                                                    // marker.pose.position.y = start_y;
                                                    // marker.pose.position.z = start_z;
                                                    // allMarker.markers.push_back(marker);

                                                    // cctt++;
                                                    // marker.id = cctt;
                                                    // marker.pose.position.x = target_x;
                                                    // marker.pose.position.y = target_y;
                                                    // marker.pose.position.z = target_z;
                                                    // allMarker.markers.push_back(marker);

                                                    // marker.color.g = 0.8f;
                                                    // marker.color.b = 0.8f;

                                                //Uncommenting this section to view the search process.
    int start_key_x = int(start_x/rz);
    int start_key_y = int(start_y/rz);
    int start_key_z = 0;
    if(start_key_x<0) start_key_x--;
    if(start_key_y<0) start_key_y--;
    start_key_x+=map_width_x/2;
    start_key_y+=map_heigh_y/2;

    int target_key_x = int(target_x/rz);
    int target_key_y = int(target_y/rz);
    int target_key_z = 0;
    if(target_key_x<0) target_key_x--;
    if(target_key_y<0) target_key_y--;
    target_key_x+=map_width_x/2;
    target_key_y+=map_heigh_y/2;

    std::set<Surf3d*,Comparexyz> find_cost;//所有的节点，包括开和关
    std::priority_queue<Surf3d*,std::vector<Surf3d*>,Comparecost> open_check;//所有的开节点
    Surf3d* currentNode(new Surf3d());

    currentNode->key_x = start_key_x;
    currentNode->key_y = start_key_y;
    int kz = -1;
    Surfel* cc =  MultiLayer_Grid[start_key_x][start_key_y];
    while(std::abs(cc->typical_z-start_z)>1.5*rz){cc = cc->next_level;kz++;}
    currentNode->key_z = kz;
    currentNode->h =(std::abs(currentNode->key_x - target_key_x)+
                         std::abs(currentNode->key_y - target_key_y)+
                         std::abs(currentNode->key_z - target_key_z));
    currentNode->g = 0;
    currentNode->cost = currentNode->g + currentNode->h;
    currentNode->is_open = true;
    currentNode->parent = nullptr;
    currentNode->fz = start_z;
    open_check.push(currentNode);
    find_cost.insert(currentNode);

        double time1 = ros::Time::now().toSec();
        bool search_flag = false;
        float target_fz = target_z; 
        while(!open_check.empty())
        {
            currentNode = open_check.top();
            open_check.pop();
            currentNode->is_open = false;//注意，只会从open_check中删除，但是不会从find_cost中删除！！！！！
            // if ((target_key_x == currentNode->key_x) && (target_key_y == currentNode->key_y))
            // std::cout<<"here"<<std::endl;
            if (
                (target_key_x == currentNode->key_x) && 
                (target_key_y == currentNode->key_y) && 
                (std::abs(target_z - currentNode->fz)<=2.5*rz)
               )
            {
                search_flag = true;
                break;
            }

            for(int i=0;i<xy_patch.size();i++)
            {
                int nei_x = xy_patch[i].first  + currentNode->key_x;
                int nei_y = xy_patch[i].second + currentNode->key_y;//邻居key

                Surfel* this_nei_node = MultiLayer_Grid[nei_x][nei_y];//邻居
                Surfel* pre_node = this_nei_node;
                int cct_z = 0;
                while(this_nei_node)
                {
                    if(std::abs(this_nei_node->typical_z - currentNode->fz)<=1.5*rz)
                    {//同层
                                        Surf3d* temp_neibour(new Surf3d());                    
                                        temp_neibour->key_x = nei_x;
                                        temp_neibour->key_y = nei_y;
                                        temp_neibour->key_z = cct_z;
                                        temp_neibour->fz = this_nei_node->typical_z;                                        
                                        if((find_cost.find(temp_neibour)!=find_cost.end()))
                                        {
                                            delete temp_neibour;
                                            break;
                                        }//这说明已经在close和open中,或者不安全
                                        else if(!check_safe(temp_neibour))
                                        {
                                            // std::cout<<"Fail pair "<<nei_x<<" "<<nei_y<<std::endl;
                                            delete temp_neibour;
                                            break;
                                        }//这说明已经在close和open中,或者不安全
                                        else
                                        {//候选扩展点
                                            temp_neibour->parent = currentNode;
                                            temp_neibour->h =  std::abs(temp_neibour->key_x-target_key_x)+
                                                               std::abs(temp_neibour->key_y-target_key_y)+
                                                               std::abs(temp_neibour->key_z-target_key_z);
                                            temp_neibour->g = currentNode->g + 1;
                                            temp_neibour->cost = temp_neibour->g + temp_neibour->h;
                                            temp_neibour->is_open = true;
                                            temp_neibour->fz = this_nei_node->typical_z;
                                            open_check.push(temp_neibour);
                                            find_cost.insert(temp_neibour);
                                                                //Uncommenting this section to view the search process.
                                                                // cctt++;
                                                                // marker.id = cctt;
                                                                // marker.pose.position.x = rz * float(temp_neibour->key_x-map_width_x/2);
                                                                // marker.pose.position.y = rz * float(temp_neibour->key_y-map_heigh_y/2);
                                                                // marker.pose.position.z = currentNode->fz;
                                                                // allMarker.markers.push_back(marker);
                                                                //Uncommenting this section to view the search process.
                                            break;
                                        }
                    }
                    else//不同层
                    {
                        pre_node = this_nei_node;
                        this_nei_node = this_nei_node->next_level;
                        cct_z++;
                    }
                } 
            }
        }
        double time2 = ros::Time::now().toSec();        
        // std::cout<<"time: "<<time2-time1<<std::endl;
        // while(1);
                                //Uncommenting this section to view the search process.
                                // Surf3d* t_node = currentNode;
                                // while(t_node!=nullptr)
                                // {
                                //     cctt++;
                                //     marker.id = cctt;
                                //     marker.pose.position.x = rz * float(t_node->key_x-map_width_x/2);
                                //     marker.pose.position.y = rz * float(t_node->key_y-map_heigh_y/2);
                                //     marker.pose.position.z = t_node->fz;
                                //     allMarker.markers.push_back(marker);
                                //     t_node = t_node->parent;
                                // }
                                // ros::Rate rzt(1);
                                // while(ros::ok())
                                // {
                                //     rzt.sleep();
                                //     test_pub.publish(allMarker);
                                // }
                                // Uncommenting this section to view the search process.            
        int step_cost = 0;
        milestone.resize(0);
        int s = 0;
        if(search_flag) 
        {
          //   geometry_msgs::PoseStamped pose_this;
          //   pose_this.header.stamp = ros::Time::now();
          //   pose_this.header.frame_id = "map";  
          //   pose_this.pose.orientation.x = 0.0;
          //   pose_this.pose.orientation.y = 0.0;
          //   pose_this.pose.orientation.z = 0.0;
          //   pose_this.pose.orientation.w = 1.0;
          path_range_min_x = INT32_MAX;
          path_range_min_y = INT32_MAX;
          path_range_max_x = INT32_MIN;
          path_range_max_y = INT32_MIN;
          std::vector<Eigen::Vector3f>all_path;
          while(currentNode)
          {
            step_cost++;
            all_path.push_back({
            rz * float(currentNode->key_x-map_width_x/2),
            rz * float(currentNode->key_y-map_heigh_y/2),
            currentNode->fz
            });
            path_range_min_x = std::min(path_range_min_x,currentNode->key_x);
            path_range_min_y = std::min(path_range_min_y,currentNode->key_y);
            path_range_max_x = std::max(path_range_max_x,currentNode->key_x);
            path_range_max_y = std::max(path_range_max_y,currentNode->key_y);
            // std::cout<<float(currentNode->key_x-200)*0.25
            //    <<" "<<float(currentNode->key_y-200)*0.25
            //    <<" "<<currentNode->fz<<std::endl;
            currentNode = currentNode->parent;
          }
            int milestone_num = float(step_cost)*rz / max_milestone_step; // 这里的step是10m默认值，所以只有
            milestone_num = std::max(milestone_num,1);
            //一共分成多少段
            if(float(milestone_num)*max_milestone_step + max_milestone_step/2.0 <=
            float(step_cost)*rz)milestone_num++;//如果段落太少适当补充
            int milestone_len = step_cost / milestone_num;//每段的长度
            milestone_len ++;//向上取整数，这样最后一个节点就可以不要，直接取终点
            // milestone.resize(std::max(milestone_num+1,2),{0.0,0.0,0.0,0.0});
            // int step_each = milestone-1
            // milestone.shrink_to_fit();
            int mid_ind = 1;
            int half_span_size = std::min(5,milestone_len/2);
            while(mid_ind * milestone_len + half_span_size <= step_cost-1)
            {
                std::vector<Eigen::Vector3f> this_seg;                        
                for(int i=mid_ind * milestone_len - half_span_size;
                        i<=mid_ind * milestone_len + half_span_size;
                        i++)
                    {
                        this_seg.push_back(all_path[step_cost-1-i]);
                    }        
                    Eigen::Vector3f this_dir = computePCA(this_seg);
                double s_v = 0;
                for(int i=1;i<this_seg.size();i++)
                {
                        Eigen::Vector3f t_dir = this_seg[i] - this_seg[i-1];
                        t_dir.normalize();
                        s_v += t_dir.dot(this_dir);
                }
                if(s_v<0) this_dir = this_dir * (-1.0);
                // std::cout<<float(this_dir(1))<<" "<<float(this_dir(0))<<std::endl;
                milestone.push_back({all_path[step_cost - 1 - mid_ind * milestone_len](0),
                                     all_path[step_cost - 1 - mid_ind * milestone_len](1),
                                     all_path[step_cost - 1 - mid_ind * milestone_len](2),
                                     std::atan2(float(this_dir(1)),float(this_dir(0)))
                                     });
                mid_ind++;
            }
            //这里要小心size = 0
            if(milestone.size()!=0)
            {
                float tx = milestone.back()[0]-target_x;
                float ty = milestone.back()[1]-target_y;
                float tz = milestone.back()[2]-target_z;
                if(std::sqrt((tx*tx)+(ty*ty)+(tz*tz))>max_milestone_step/2.0)
                milestone.push_back({target_x, target_y, target_z, target_yaw});
                else
                milestone[milestone.size()-1] = {target_x, target_y, target_z, target_yaw};//最后一个直接覆盖
            }
            else
            milestone.push_back(
            {target_x, 
             target_y, 
             target_z, 
             target_yaw}
            );//最后一个直接覆盖            
        }
        for(std::set<Surf3d*,Comparexyz>::iterator it = find_cost.begin();it!=find_cost.end();it++)
        delete *it;
        if(search_flag)return step_cost;
        return -1;
}


bool AstarSurPlanner::check_safe(Surf3d* this_node)
{
    for(int i=(0-check_safe_size);i<=check_safe_size;i++)
    {
        for(int j=(0-check_safe_size);j<=check_safe_size;j++)
        {
            int nei_x = this_node->key_x + i;
            int nei_y = this_node->key_y + j;//邻居key
            Surfel* this_nei = MultiLayer_Grid[nei_x][nei_y];//邻居
            while(this_nei)
            {
                if(std::abs(this_nei->typical_z - this_node->fz)<=1.5*rz)
                {//同层
                    if(this_nei->status == -1)return false;
                    else{break;}//如果安全考虑其他邻居 
                }
                else//不同层
                {
                    this_nei = this_nei->next_level;
                }
            } 
            if(this_nei == nullptr)return false;
        }
    }
    return true;
}

Eigen::Vector3f AstarSurPlanner::computePCA(const std::vector<Eigen::Vector3f>& points) 
{
    // 计算均值
    Eigen::Vector3f meanv = Eigen::Vector3f::Zero();
    for (const auto& point : points) {
        meanv += point;
    }
    meanv /= static_cast<float>(points.size());

    // 计算协方差矩阵
    Eigen::Matrix3f covariance = Eigen::Matrix3f::Zero();
    for (const auto& point : points) {
        Eigen::Vector3f centered = point - meanv;
        covariance += centered * centered.transpose();
    }
    covariance /= static_cast<float>(points.size() - 1);
    // 计算特征值和特征向量
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> es(covariance);

    Eigen::Vector3f vec = es.eigenvalues();
    int maxIndex = 0;
    float maxValue = vec(0);
    for(int i=0;i<vec.size();i++)
    {
        if(maxValue < vec(i))
        {
            maxIndex = i;
            maxValue = vec(i);
        }
    }
    // 返回最大特征值对应的特征向量（‌主成分）‌
    return es.eigenvectors().col(maxIndex);
}