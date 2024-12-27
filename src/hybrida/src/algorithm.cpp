 
#include "algorithm.hpp"
using namespace HybridAStar;

Algorithm::Algorithm(int width_x_, int heigh_y_, float res_, Surfel*** MultiLayer_Grid_,ros::Publisher& tp_b_,
ros::Publisher& tp_b1_)
{
  width_x = width_x_;
  heigh_y = heigh_y_;
  res = res_;
  MultiLayer_Grid = MultiLayer_Grid_;
  tp_b = tp_b_;  
  tp_b1 = tp_b1_;  
}

std::pair<bool,float> Algorithm::Caculate_z(float w_x,float w_y,float fuzzy_z)
{
    //返回此处的z和安全标识
    std::pair<bool,float> return_pair;
    return_pair.first = false;
    return_pair.second = fuzzy_z;
    int key_x = int(w_x/res) + width_x/2;
    int key_y = int(w_y/res) + heigh_y/2;
    if(w_x<0) key_x--;
    if(w_y<0) key_y--;
    int k_z = 0;
    Surfel* this_node = SearchinMap(w_x,w_y,fuzzy_z,k_z);
    if(this_node==nullptr)
    return_pair.first = false;//未知，无效
    if(this_node)
    {
        if(this_node->status==HybridAStar::UNTRA)
        return_pair.first = false;//如果地图不可以行驶，无效
        else
        {
            return_pair.first = true;
            return_pair.second = (this_node->a * w_x + this_node->b * w_y + this_node->d) / (0-this_node->c);
        }
    }
    return return_pair;
}
float Algorithm::CaculateG(float pre_g,int pre_prim,int now_prim)
{
  float now_g = pre_g;
  if (now_prim < 3) 
  {//前进情况
    if (pre_prim != now_prim) 
    {//方向发生改变时
      if (pre_prim > 2) 
      {
        now_g += dx[0] * Constants::penaltyTurning * Constants::penaltyCOD;//改变方向的惩罚  1.47
      } else 
      {
        now_g += dx[0] * Constants::penaltyTurning;//没有改变方向  0.735
      }
    } 
    else 
    {//方向没有发生变化
      now_g += dx[0];//0.7
    }
  }
  else 
  {//后退
    if (pre_prim != now_prim) 
    {
      if (pre_prim < 3) 
      {
        now_g += dx[0] * Constants::penaltyTurning * Constants::penaltyReversing * Constants::penaltyCOD;
      }
      else 
      {
        now_g += dx[0] * Constants::penaltyTurning * Constants::penaltyReversing;
      }
    }
    else 
    {
      now_g += dx[0] * Constants::penaltyReversing;
    }
  }
  return now_g;
}


void Algorithm::Coor2KeyXY(float w_x,float w_y,int& key_x,int& key_y)
{
  key_x = int(w_x/res) + width_x/2;
  key_y = int(w_y/res) + heigh_y/2;
  if(w_x<0) key_x--;
  if(w_y<0) key_y--;  
}
Surfel* Algorithm::SearchinMap(float w_x,float w_y,float w_z,int& z_layer)
{
  int key_x;
  int key_y;
  Coor2KeyXY(w_x,w_y,key_x,key_y);
  Surfel* this_node = MultiLayer_Grid[key_x][key_y];
  if(this_node==nullptr) return nullptr;
  Surfel* pre_node = this_node;

  int cct = 0;
  while(this_node)
  {
    if(std::abs(this_node->typical_z - w_z) <= sam_z_thr)
    {
      break;
    }
    else
    {
      pre_node = this_node;
      this_node = this_node->next_level;
      cct++;
    }
  } 
  z_layer = cct;
  return this_node; 
}

//在地图中进行搜索节点，如果地图中有这个节点则返回key，并且计算这个地方的key和对应的float_z,
//如果不存在节点或者不安全返回false
bool Algorithm::Coor2KeyXYZYaw(float w_x,float w_y,float w_z,float w_yaw,
int& key_x,int& key_y,int& key_z,int& key_yaw,float& proj_z)
{
  key_x = int(w_x/res) + width_x/2;
  key_y = int(w_y/res) + heigh_y/2;
  if(w_x<0) key_x--;
  if(w_y<0) key_y--;  

  Surfel* this_node = SearchinMap(w_x,w_y,w_z,key_z);
  if(this_node==nullptr) return false;//未知，无效
  if(this_node)
  {
    if(this_node->status==HybridAStar::UNTRA)return false;//如果地图不可以行驶，有效
    key_yaw = int(w_yaw / yaw_res);    
    proj_z = (this_node->a * w_x + this_node->b * w_y + this_node->d) / (0-this_node->c);
    return true;//如果地图可以行驶，有效
  }
  else return false; //未知，无效
}

//计算下一个节点，如果节点不安全，直接Null
//如果安全： 就返回新建的节点  
//         传入当前节点的空间信息，对应的surfel信息
//this_node: 离散空间key以及精准的坐标信息
//     abcd: 离散空间对应的surfel信息 要求：abc已经归一化
Node3D* Algorithm::createSucc(Node3D* this_node,float a,float b,float c,float d,int prim_index)
{
  //根据轨迹计算下一个节点所在的空间位置
  float xSucc;
  float ySucc;
  float zSucc;
  float tSucc;

  float x = this_node->float_x;
  float y = this_node->float_y;
  float z = this_node->float_z;
  float t = this_node->t;

  //这里在搜索的时候没有利用平面信息进行投影！！！
  if (prim_index < 3)
  {//前向 Successor
    xSucc = 0 + dx[prim_index] * cos(t) - dy[prim_index] * sin(t);
    ySucc = 0 + dx[prim_index] * sin(t) + dy[prim_index] * cos(t);
    tSucc = Helper::normalizeHeadingRad(t + dt[prim_index]);
    //setpred的时候是否改变运动prim?
  }
  else 
  {//后向 Successor
    xSucc = 0 - dx[prim_index - 3] * cos(t) - dy[prim_index - 3] * sin(t);
    ySucc = 0 - dx[prim_index - 3] * sin(t) + dy[prim_index - 3] * cos(t);//如果此处认为旋转矩阵不变，相当于dx被加了一负号
    switch (prim_index)
    {
      case 3:
      tSucc = Helper::normalizeHeadingRad(t + dt[0]);
        break;
      case 4:
      tSucc = Helper::normalizeHeadingRad(t + dt[2]);
        break;    
      case 5:
      tSucc = Helper::normalizeHeadingRad(t + dt[1]);
        break;
    }
  }
  //目前xytSucc是surfel坐标系下的结果，现在将结果转换到world坐标系下
  //求解原点在surfel平面上的投影
  float tem_org[3] = {-a*d,-b*d,-c*d};//世界坐标系原点在surfel上投影
  float tem_orgx[3] = {1-a*(a+d),-b*(a+d),-c*(a+d)};//世界坐标系(1，0，0)在surfel上投影
  tem_orgx[0] -= tem_org[0];
  tem_orgx[1] -= tem_org[1];
  tem_orgx[2] -= tem_org[2];////surfel x轴方向在world下的表示
  float tem_tt = sqrt(tem_orgx[0]*tem_orgx[0]+tem_orgx[1]*tem_orgx[1]+tem_orgx[2]*tem_orgx[2]);
  float x_axis[3] = {tem_orgx[0]/tem_tt,tem_orgx[1]/tem_tt,tem_orgx[2]/tem_tt};//surfelx轴单位向量在world下的表示
  float z_axis[3] = {a,b,c};//surfelz轴单位向量在world下的表示
  float y_axis[3] = {
                     z_axis[1]*x_axis[2]-z_axis[2]*x_axis[1],
                     z_axis[2]*x_axis[0]-z_axis[0]*x_axis[2],
                     z_axis[0]*x_axis[1]-z_axis[1]*x_axis[0]
                    };//surfely轴单位向量在world下的表示
  //T12
  xSucc *=res;  ySucc *=res;
  //原先的
  // float exp_x = x_axis[0] * xSucc + x_axis[1] * ySucc + x_axis[2] * 0 + x;
  // float exp_y = y_axis[0] * xSucc + y_axis[1] * ySucc + y_axis[2] * 0 + y;
  // float exp_z = z_axis[0] * xSucc + z_axis[1] * ySucc + z_axis[2] * 0 + z;
  //修正的
  // float exp_x = x_axis[0] * xSucc + y_axis[0] * ySucc + z_axis[0] * 0 + x;
  // float exp_y = x_axis[1] * xSucc + y_axis[1] * ySucc + z_axis[1] * 0 + y;
  // float exp_z = x_axis[2] * xSucc + y_axis[2] * ySucc + z_axis[2] * 0 + z;
  float exp_x = xSucc + x;
  float exp_y = ySucc + y;
  float exp_z = z;
  float exp_yaw = tSucc;
  //将局部坐标系下的点转移到全局坐标系下
  //我们认为tSucc的变化不大

  //判断是否已经在find_set中，如果已经在，就退出
  //在地图中判定该空间位置是否安全，如果安全就创建并且返回
  int nx_kx,nx_ky,nx_kya,nx_kzl;//算key
  float nx_ty_z;
  bool search_succ = Coor2KeyXYZYaw(exp_x,exp_y,exp_z,exp_yaw,nx_kx,nx_ky,nx_kzl,nx_kya,nx_ty_z);//扩展节点是否安全？
  if(search_succ)//扩展节点安全
  {
    bool on_the_ground = false;
    on_the_ground =  Check_Four_Wheels(exp_x,exp_y,exp_z,exp_yaw);
    // on_the_ground = true;
    if(on_the_ground)//保证四轮着陆
    {
      Node3D* next_node = new Node3D(nx_kx,nx_ky,nx_kzl,nx_kya,prim_index,exp_x,exp_y,nx_ty_z,exp_yaw);
      return next_node;
    }
    else return nullptr;
  }
  else //扩展节点不安全
  return nullptr; 
}
bool Algorithm::isreach(Node3D* this_node,int target_k_x,int target_k_y,float target_z,
int target_k_yaw)
{
  if(std::abs(target_k_x-this_node->grid_x) + 
     std::abs(target_k_y-this_node->grid_y) + 
     std::abs(target_k_yaw-this_node->yaw_index)<=1
  && std::abs(target_z-this_node->float_z)<0.25
    )return true;
    return false;
  // if(std::abs(target_k_x-this_node->grid_x) + 
  //    std::abs(target_k_y-this_node->grid_y) <=1
  // && std::abs(target_z-this_node->float_z)<0.25
  // &&std::abs(target_k_yaw-this_node->yaw_index)<=2
  //   )return true;
  //   return false;
}
float Algorithm::MoveCost(Node3D* node1,Node3D* node2)
{
  return
  std::sqrt(
                (node1->float_x-node2->float_x)*(node1->float_x-node2->float_x) +
                (node1->float_y-node2->float_y)*(node1->float_y-node2->float_y) + 
                (node1->float_z-node2->float_z)*(node1->float_z-node2->float_z)
           );
}
bool Algorithm::SameKey(Node3D* node1,Node3D* node2)
{
  if(node1->grid_x == node2->grid_x && node1->grid_y == node2->grid_y && 
     node1->z_layer == node2->z_layer && node1->yaw_index == node2->yaw_index)
  return true;
  return false;
}

void Algorithm::ClearFrontDataSet()
{
  for(std::set<Node3D *, HybridAStar::CompareNode3d>::iterator itt=find_set.begin();itt!=find_set.end();itt++)
  delete *itt;
  find_set.clear();
  open_list.clear();
}
//返回的单位是grid

// tp_b
float Algorithm::CaculateH(float now_x,float now_y,float now_z,float now_yaw,
float target_x,float target_y,float target_z,float target_yaw)
{
  float heuristic_cost = 0;

  AstarSurPlanner astar_handle(MultiLayer_Grid,width_x,heigh_y,res,tp_b);
  heuristic_cost =  astar_handle.AstarPlan(now_x,now_y,now_z,target_x,target_y,target_z);
  if(heuristic_cost<0)
  while(1)std::cout<<"no solution! check start posture and target posture!"<<std::endl; 

  float dubinsCost = 0;
  float reedsSheppCost = 0;
  if (Constants::usedubins) 
  {
    DubinsPath path;
    double q0[] = {now_x,now_y,now_yaw};
    double q1[] = {target_x,target_y,target_yaw};
    dubins_init(q0, q1,Constants::r,&path);//q0 q1的单位是m,r的单位是m
    dubinsCost = dubins_path_length(&path);
    dubinsCost /= res;
    //这里改用open motion planning library的算法 
    // ompl::base::DubinsStateSpace dubinsPath(float(Constants::r)/res);
    // State* dbStart = (State*)dubinsPath.allocState();
    // State* dbEnd = (State*)dubinsPath.allocState();
    // //这里的单位是grid，所以上面的半径需要换算
    // dbStart->setXY(now_x / res, now_y / res);
    // dbStart->setYaw(now_yaw);
    // dbEnd->setXY(target_x / res, target_y / res);
    // dbEnd->setYaw(target_yaw);
    // //这里的单位是grid，所以上面的半径需要换算
    // dubinsCost = dubinsPath.distance(dbStart, dbEnd);
    //杜斌曲线的距离是grid，而不是m
  }

  return std::max(heuristic_cost,dubinsCost);
}
void Algorithm::Key2Coor(int kx,int ky,float& coorx,float& coory)
{
  coorx = float(kx-width_x/2) * res + res/2.0;
  coory = float(ky-heigh_y/2) * res + res/2.0;
}
 
bool Algorithm::CheckTraversable(Node3D* this_node)
{
  //根据车头的方向进行扩展，前后长度为6，根据车y的长度进行扩展，左右长度为6 
  float v_x = std::cos(this_node->t);
  float v_y = std::sin(this_node->t);
  float v_z = 0;

  v_x = v_x - this_node->float_x;
  v_y = v_y - this_node->float_y;
  v_z = v_z - this_node->float_z;
  float a,b,c,d;int z_l;
  Surfel* this_sur = SearchinMap(this_node->float_x,this_node->float_y,this_node->float_z,z_l);
  if(this_sur==nullptr)return false;
  if(this_sur->status == HybridAStar::UNTRA)return false;
  
  a = this_sur->a;
  b = this_sur->b;
  c = this_sur->c;
  d = this_sur->d;  
  float tem_tt = (0-a*v_x-b*v_y-c*v_z-d)/1.0;
  v_x += tem_tt * a; v_y += tem_tt * b; v_z += tem_tt * c;  //直线向量方向
  v_x /=std::sqrt(v_x*v_x+v_y*v_y+v_z*v_z);
  v_y /=std::sqrt(v_x*v_x+v_y*v_y+v_z*v_z);
  v_z /=std::sqrt(v_x*v_x+v_y*v_y+v_z*v_z);

  float v_x_v = v_y*c - v_z*b;
  float v_y_v = v_z*a - v_x*c;
  float v_z_v = v_x*b - v_y*a;

  //x方向7个锚点
  for(int i = -3;i<=3;i++)
  {
    float x_line = this_node->float_x+v_x*res;
    float y_line = this_node->float_y+v_y*res;
    float z_line = this_node->float_z+v_z*res;
    for(int j=-2;j<=2;j++)
    {

      float x_vert = x_line + v_x_v * res;
      float y_vert = y_line + v_y_v * res;
      float z_vert = z_line + v_z_v * res;
      this_sur = SearchinMap(x_vert,y_vert,z_vert,z_l);
      if(this_sur==nullptr)return false;
      if(this_sur->status == HybridAStar::UNTRA)return false;
    }
  }
  return true;
}

//返回杜宾曲线段的头和尾,搜索得到的节点有x y z yaw
std::pair<Node3D*,Node3D*> Algorithm::dubinsShot(float now_x,float now_y,float now_z,float now_yaw,float target_x,float target_y,float target_z,float target_yaw)
{
  double q0[] = {now_x,now_y,now_yaw};
  double q1[] = {target_x,target_y,target_yaw};

  DubinsPath path;
  dubins_init(q0, q1,Constants::r,&path);//q0 q1的单位是m,r的单位是m
  int i = 0;
  float x = Constants::dubinsStepSize;//放掉第一个节点，因为重复了
  float length = dubins_path_length(&path);
  if(length < sqrt((now_x-target_x)*(now_x-target_x)
                  +(now_y-target_y)*(now_y-target_y)
                  +(now_z-target_z)*(now_z-target_z)
                  )/2.0)
  {return {nullptr,nullptr};}
  Node3D* pre_n=nullptr;  Node3D* now_n=nullptr;
  Node3D* dubinsNodes = new Node3D [int(length / Constants::dubinsStepSize)];
  float this_ref_z = now_z;
  while (x <  length)
  {
    double q[3];
    dubins_path_sample(&path, x, q);//从起点到终点前进x m，得到一个采样点 q
    dubinsNodes[i].float_x = q[0];
    dubinsNodes[i].float_y = q[1];
    dubinsNodes[i].t = q[2];
    bool find_surf = Coor2KeyXYZYaw(q[0],q[1],this_ref_z,q[2],
    dubinsNodes[i].grid_x,dubinsNodes[i].grid_y,dubinsNodes[i].z_layer,dubinsNodes[i].yaw_index,
    dubinsNodes[i].float_z);
    if(find_surf)
    this_ref_z=dubinsNodes[i].float_z;
    else 
    {
      delete [] dubinsNodes;
      return {nullptr,nullptr};
    }
    if (CheckTraversable(&dubinsNodes[i])) 
    {
      pre_n = now_n;
      now_n = &dubinsNodes[i];
      now_n->prim = -1;
      now_n->parent = pre_n;
      x += Constants::dubinsStepSize;
      i++;
      if(i>1)
      {
        float delta_yaw_this =now_n->t - now_n->parent->t;
        if(delta_yaw_this>M_PI)delta_yaw_this = delta_yaw_this-2*M_PI;
        else if(delta_yaw_this<-M_PI)delta_yaw_this = delta_yaw_this+2*M_PI;
        if(std::abs(delta_yaw_this)<=0.015)now_n->prim = 0;
        else if(delta_yaw_this>0) now_n->prim = 1;
        else now_n->prim = 2;        
      }

    }
    else
    {
      delete [] dubinsNodes;
      return {nullptr,nullptr};
    }
    //这里的单位是grid
    //杜斌曲线中，没有对prim运动源语进行设定！！！！！！
    //跳出循环的条件之二：生成的路径存在碰撞节点
  }
  // Node3D* t_n = &dubinsNodes[(int)(length / Constants::dubinsStepSize)];
  // while(t_n)
  // {
  //   t_n->float_z =0;
  //   t_n = t_n->parent;
  // }
  return {&dubinsNodes[0],&dubinsNodes[int(length / Constants::dubinsStepSize)-1]};
}
//1、从open_list 拿到头节点h_n,并从open_list中删除
//2、如果 h_n.is_close continue
//3、h_n.is_close = true;
//4、h_n 从6个方向进行扩展 nei_n
//5、一个方向计算后从地图中查找得到nei_n 的 z_layer 和 traversable
//6、find_set查找在不在
//        不在就添加到 find_set和open_list中
//7、
//A1、find_set查找nei_n在，是open状态，并且id和h_n不同
    //A2-1、沿着当前这条路径进行扩展（即经过h_n）不能降低他的代价G，结束
    //A2-2、沿着当前这条路径进行扩展（即经过h_n）能降低他的代价G，那就计算nei_n当前和之前路径的G+H，如果降低就把nei_n的父节点设定为h_n，
    //更新open_list（因为代价改变了）

//B1、如果nei_n和h_n一样 代表在同一个grid中并且角度误差为5度内
    // B2-1、沿着当前这条路径进行扩展（即经过nPred）不能降低他的代价G+H,放弃
    // B2-2、沿着当前这条路径进行扩展（即经过nPred）能降低他的代价G+H,则更新nei_n->parent = h_n->parent,并且对h_n扩展结束以后，
    // 需要用nei_n替代h_
bool Algorithm::PrepareNavi(float now_x,float now_y,float now_z,float now_yaw,
float target_x,float target_y,float target_z,float target_yaw,vector<vector<float>>& milestone)
{
  AstarSurPlanner astar_handle(MultiLayer_Grid,width_x,heigh_y,res,tp_b);
  int heuristic_cost_grid =  astar_handle.GetMilestones(now_x,now_y,now_z,now_yaw,
  target_x,target_y,target_z,target_yaw,milestone);
  if(heuristic_cost_grid<0)while(1)std::cout<<"error"<<std::endl; 
  return true;
}
Node3D* Algorithm::hybridAStar(float start_x,float start_y,float start_z,
float start_yaw,float target_x,float target_y,float target_z,float target_yaw)
{
  
  int   start_k_x = 0;    int target_k_x = 0;    int now_k_x = 0;    
  int   start_k_y = 0;    int target_k_y = 0;    int now_k_y = 0;    
  int   start_k_z = 0;    int target_k_z = 0;    int now_k_z = 0;    
  int   start_k_yaw = 0;  int target_k_yaw = 0;  int now_k_yaw = 0;  
  float start_p_z = 0.0;float target_p_z = 0.0;
  if(!Coor2KeyXYZYaw(start_x,start_y,start_z,start_yaw,start_k_x,start_k_y,start_k_z,start_k_yaw,start_p_z))
  return nullptr;
  if(!Coor2KeyXYZYaw(target_x,target_y,target_z,target_yaw,target_k_x,target_k_y,target_k_z,target_k_yaw,target_p_z))
  return nullptr;
  Node3D* succ_node;
  Node3D* this_node;
  this_node = new Node3D(start_k_x,start_k_y,start_k_z,start_k_yaw,0,start_x,start_y,start_z,start_yaw);
  this_node->g = 0;
  this_node->h = CaculateH(start_x,start_y,start_z,start_yaw,target_x,target_y,target_z,target_yaw);  
  this_node->is_open = true;
  find_set.insert(this_node);
  open_list.insert(this_node);
  int hybridA_iteration = 0;
  
  // VisualMark(start_x,start_y,start_z,start_yaw,0);
  // VisualMark(target_x,target_y,target_z,target_yaw,0);

  while(!open_list.empty())
  {
    find_itt this_itt = open_list.begin();
    if((*this_itt)->is_open==false)
    {open_list.erase(this_itt);continue;}
    //如果已经关闭直接删掉
    this_node = *this_itt;
    this_node->is_open = false;
    open_list.erase(this_itt);
    //开始对周围节点进行扩展
    // if(this_node->grid_x==192 && this_node->grid_y==201
    // && this_node->yaw_index==-29)
    // if(this_node->prim==2)
    // {
    //   std::cout<<"here"<<std::endl;
    //   VisualTra(this_node);
    // }
    hybridA_iteration++;
    // cout<<hybridA_iteration<<" / "<<HybridAStar::MAXITERATION<<std::endl;
    // if(this_node->grid_x==180 && this_node->grid_y==188 && 
    //    this_node->z_layer==0 && this_node->yaw_index==-12)
    // std::cout<<this_node->grid_x<<" "<<this_node->grid_y<<" "<<this_node->yaw_index<<std::endl;

    // if(this_node->float_x<=-5 && this_node->float_x>=-5.7 && 
    //    this_node->float_y<=-2.5 && this_node->float_y>=-3.2)
    // std::cout<<this_node->grid_x<<" "<<this_node->grid_y<<" "<<this_node->yaw_index<<std::endl;
    // VisualMark(this_node->float_x,this_node->float_y,this_node->float_z,this_node->t,this_node->prim);
    if (isreach(this_node,target_k_x,target_k_y,target_z,target_k_yaw) 
    || hybridA_iteration > HybridAStar::MAXITERATION)
    {
        if(hybridA_iteration > HybridAStar::MAXITERATION)   
        {
          std::cout<<"iterations out!!!"<<std::endl;
          ros::Rate ttt(1);
          while(ros::ok())
          {
            VisualMarkwithscore();
            // tp_b.publish(allMarker); 
            ttt.sleep();
          }
          return nullptr;//到达
        }
        return this_node;
    }
    //尝试使用杜宾曲线
    // if(0)
    if(this_node->prim < 3 && this_node->h<=40)
    {
      std::pair<Node3D*,Node3D *> dubin_seg;
      dubin_seg =  dubinsShot(this_node->float_x,this_node->float_y,this_node->float_z,this_node->t,
                              target_x,target_y,target_z,target_yaw);
      if(dubin_seg.first!=nullptr)
      {
        dubin_seg.first->parent = this_node;//杜宾搜索非空，将头连接到历史路径中
                      float delta_yaw_this = dubin_seg.first->t - this_node->t;
                      if(delta_yaw_this>M_PI)delta_yaw_this = delta_yaw_this-2*M_PI;
                      else if(delta_yaw_this<-M_PI)delta_yaw_this = delta_yaw_this+2*M_PI;
                      if(std::abs(delta_yaw_this)<=0.015)dubin_seg.first->prim = 0;
                      else if(delta_yaw_this>0) dubin_seg.first->prim = 1;
                      else dubin_seg.first->prim = 3;        
        return dubin_seg.second;//返回尾部元素
      }
    }
    ccincre = 0;
    // increMarker.markers.clear();
    // VisualIncre(this_node,true);
    for (int i = 0; i < 6; i++) 
    {
      Surfel* this_sur = SearchinMap(this_node->float_x,this_node->float_y,this_node->float_z,this_node->z_layer);
      succ_node = createSucc(this_node,this_sur->a,this_sur->b,this_sur->c,this_sur->d,i);
      // VisualMarkwithscore();
      // if(succ_node!=nullptr)
      // VisualIncre(succ_node,false);
      //内部业务逻辑已经简化，节点需要额外检查
      if(succ_node==nullptr)continue;//扩展节点不安全
      // std::cout<<"succ_node: "<<succ_node->float_x<<" "<<succ_node->float_y<<" "<<succ_node->float_z<<std::endl;
      //先处理最简单的情况，就是新节点没有出现过 C1
      if(find_set.find(succ_node)==find_set.end())
      {
        //计算节点的g h c 
        succ_node->h = CaculateH(succ_node->float_x,succ_node->float_y,succ_node->float_z,succ_node->t,
        target_x,target_y,target_z,target_yaw);
        succ_node->g = CaculateG(this_node->g,this_node->prim,succ_node->prim);
        succ_node->parent = this_node;
        succ_node->is_open = true;
        open_list.insert(succ_node);
        find_set.insert(succ_node);
        // VisualMark(succ_node->float_x,succ_node->float_y,succ_node->float_z,succ_node->t,succ_node->prim);
        continue;
      }
      else if(!SameKey(succ_node,this_node))
      {//A1（扩展节点不新，已经在open_list中存在了）
        // float tem_g = this_node->g + MoveCost(this_node,succ_node);
        float tem_g = CaculateG(this_node->g,this_node->prim,succ_node->prim);//这条扩展路径的g
        if(tem_g>=(*(find_set.find(succ_node)))->g)//先前扩展路径的g更小
        {//A2-1
          delete succ_node;
          continue;
        }
        else
        { //A2-2 //出现断层弧线的主要原因
          delete succ_node;
          continue;
          Node3D* already_node = *(find_set.find(succ_node));//先前扩展节点
          open_list.erase(already_node);
          find_set.erase(already_node);

          already_node->parent = this_node;
          already_node->g = tem_g;
          already_node->is_open = true;//already_node可能是其他节点的parent
          already_node->float_x = succ_node->float_x;
          already_node->float_y = succ_node->float_y;
          already_node->float_z = succ_node->float_z;
          already_node->t = succ_node->t;
          already_node->z_layer = succ_node->z_layer;
          already_node->prim = succ_node->prim;
          //key就不需要赋值了因为是一样的
          open_list.insert(already_node);
          find_set.insert(already_node);//因为要重新排序
          delete succ_node;
          continue;
        }
      }
      else
      {///B1、如果nei_n和h_n一样 代表在同一个grid中并且角度误差为5度内
        float tem_g = CaculateG(this_node->g,this_node->prim,succ_node->prim);//这条扩展路径的g
        // float tem_h = CaculateH(succ_node->float_x,succ_node->float_y,succ_node->float_z,succ_node->t,
        // target_x,target_y,target_z,target_yaw);
        float tem_h = this_node->h;
        // std::cout<< "pre_node is "<<(*(find_set.find(this_node)))->h + 
        // (*(find_set.find(this_node)))->g + Constants::tieBreaker<<std::endl;
        // std::cout<< "now_cost is "<<tem_h+tem_g<<std::endl;
        if((tem_h+tem_g >= (*(find_set.find(this_node)))->h + (*(find_set.find(this_node)))->g + Constants::tieBreaker))
        {
            delete succ_node;
            continue;
        }
        else
        {
          Node3D* already_node = *(find_set.find(this_node));//先前扩展节点
          open_list.erase(already_node);
          find_set.erase(already_node);

          already_node->g = tem_g;
          already_node->h = tem_h;

          already_node->is_open = true;//already_node可能是其他节点的parent
          already_node->float_x = succ_node->float_x;
          already_node->float_y = succ_node->float_y;
          already_node->float_z = succ_node->float_z;
          already_node->t = succ_node->t;
          already_node->z_layer = succ_node->z_layer;
          //key就不需要赋值了因为是一样的
          open_list.insert(already_node);
          find_set.insert(already_node);//因为要重新排序
          delete succ_node;
          continue;
        }
      }
    }
    // std::cout<<"----------------------------------"<<std::endl;

  }
  VisualMarkwithscore();
  return nullptr;
}
void Algorithm::VisualTra(Node3D* thisn)
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time::now();
  marker.ns = "now_search";
  marker.type = visualization_msgs::Marker::ARROW;
  marker.action = visualization_msgs::Marker::ADD;

  marker.color.a = 1.0;
  marker.lifetime = ros::Duration(30.0);/////marker的生存时间只有1s
  marker.scale.x = 0.25;
  marker.scale.y = 0.02;
  marker.scale.z = 0.02;
      marker.color.r = 1.0;
      marker.color.g = 1.0;
      marker.color.b = 1.0;

    allMarker.markers.clear();
    cctt = 0;
    while(thisn)
    {
      cctt++;
      marker.id = cctt;
      marker.pose.position.x = thisn->float_x;
      marker.pose.position.y = thisn->float_y;
      marker.pose.position.z = thisn->float_z;
      geometry_msgs::Quaternion q;//初始化四元数（geometry_msgs类型）
      q=tf::createQuaternionMsgFromRollPitchYaw(0,0,thisn->t);//欧拉角转四元数（geometry_msgs::Quaternion）
      marker.pose.orientation = q;
      allMarker.markers.push_back(marker);
      thisn = thisn->parent;
    }  
      tp_b.publish(allMarker); 
      tp_b.publish(allMarker); 
}

void Algorithm::VisualIncre(Node3D* thisn,bool isc)
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time::now();
  marker.ns = "now_incre";
  marker.type = visualization_msgs::Marker::ARROW;
  marker.action = visualization_msgs::Marker::ADD;

  marker.color.a = 1.0;
  marker.lifetime = ros::Duration(10.0);/////marker的生存时间只有1s
  marker.scale.x = 0.25;
  marker.scale.y = 0.02;
  marker.scale.z = 0.02;
  marker.color.r = 1.0;
  marker.color.g = 0.0;
  marker.color.b = 0.0;

  if(isc)
  {
    marker.lifetime = ros::Duration(30.0);/////marker的生存时间只有1s
  marker.color.r = 0.0;
  marker.color.g = 0.0;
  marker.color.b = 1.0;
  }

  allMarker.markers.clear();
  ccincre++;
  marker.id = ccincre;
  marker.pose.position.x = thisn->float_x;
  marker.pose.position.y = thisn->float_y;
  marker.pose.position.z = thisn->float_z+0.1;
  geometry_msgs::Quaternion q;//初始化四元数（geometry_msgs类型）
  q=tf::createQuaternionMsgFromRollPitchYaw(0,0,thisn->t);//欧拉角转四元数（geometry_msgs::Quaternion）
  marker.pose.orientation = q;
  increMarker.markers.push_back(marker);
  thisn = thisn->parent;
  tp_b1.publish(increMarker); 
}
void Algorithm::VisualMarkwithscore()
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time::now();
  marker.ns = "now_search";
  marker.type = visualization_msgs::Marker::ARROW;
  marker.action = visualization_msgs::Marker::ADD;

  marker.color.a = 1.0;
  marker.lifetime = ros::Duration(10.0);/////marker的生存时间只有1s
  marker.scale.x = 0.25;
  marker.scale.y = 0.02;
  marker.scale.z = 0.02;

    allMarker.markers.clear();
    cctt = 0;
    for(std::set<Node3D*,CompareNode3d>::iterator itt=find_set.begin();
    itt!=find_set.end();itt++)
    {
      float ratio = float(std::distance(find_set.begin(), itt)) / find_set.size(); 
      marker.color.r = 1.0;
      marker.color.g = 1.0;
      marker.color.b = 1.0;
      cctt++;
      marker.id = cctt;
      marker.pose.position.x = (*itt)->float_x;
      marker.pose.position.y = (*itt)->float_y;
      marker.pose.position.z = (*itt)->float_z;
      geometry_msgs::Quaternion q;//初始化四元数（geometry_msgs类型）
      q=tf::createQuaternionMsgFromRollPitchYaw(0,0,(*itt)->t);//欧拉角转四元数（geometry_msgs::Quaternion）
      marker.pose.orientation = q;
      allMarker.markers.push_back(marker);
    }  
      ros::Rate ttt(0.5);
      // if(open_list.size()>40)
      // while(1)
      // {
      //   ttt.sleep();
      // tp_b.publish(allMarker); 
      // }
      tp_b.publish(allMarker); 
      tp_b.publish(allMarker); 
}

void Algorithm::VisualMark(float w_x,float w_y, float w_z,float yaw,int prim_t)
{
  //   allMarker.markers.clear();
  visualization_msgs::Marker marker;
  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time::now();
  marker.ns = "now_search";
  marker.type = visualization_msgs::Marker::ARROW;
  // marker.type = visualization_msgs::Marker::CUBE;
  marker.action = visualization_msgs::Marker::ADD;

  marker.color.r = 0.8f;
  marker.color.g = 0.0f;
  marker.color.b = 0.0f;
  cctt++;
  marker.id = cctt;
  marker.color.a = 1.0;
  marker.lifetime = ros::Duration(3);/////marker的生存时间只有1s
  marker.scale.x = 0.25;
  // marker.scale.x = 0.02;
  marker.scale.y = 0.02;
  marker.scale.z = 0.02;

  marker.pose.position.x = w_x;
  marker.pose.position.y = w_y;
  marker.pose.position.z = w_z;

  geometry_msgs::Quaternion q;//初始化四元数（geometry_msgs类型）
  q=tf::createQuaternionMsgFromRollPitchYaw(0,0,yaw);//欧拉角转四元数（geometry_msgs::Quaternion）
  marker.pose.orientation = q;

  if(allMarker.markers.size()>0)
  {
    allMarker.markers[allMarker.markers.size()-1].color.r = 0.8;
    allMarker.markers[allMarker.markers.size()-1].color.g = 0.8;
    allMarker.markers[allMarker.markers.size()-1].color.b = 0.8;
    allMarker.markers[allMarker.markers.size()-1].pose.position.z += 0.05;
  }
  allMarker.markers.push_back(marker);
  // allMarker.markers[allMarker.markers.size()-1].color.r = 0.0;
  // allMarker.markers[allMarker.markers.size()-1].color.g = 1.0;
  // allMarker.markers[allMarker.markers.size()-1].color.b = 0.0;
  // allMarker.markers[allMarker.markers.size()-1].pose.position.z -= 0.05;
  tp_b.publish(allMarker); 
  tp_b.publish(allMarker); 
  tp_b.publish(allMarker); 
  tp_b.publish(allMarker); 
}

bool Algorithm::Check_Four_Wheels(float w_x,float w_y,float w_z,float w_yaw)
{
  float corner1_x;float corner1_y;std::pair<bool,float> corner1_z;
  corner1_x = w_x + car_size_x_2*std::cos(w_yaw) - car_size_y_2*std::sin(w_yaw);
  corner1_y = w_y + car_size_x_2*std::sin(w_yaw) + car_size_y_2*std::cos(w_yaw);
  corner1_z = Caculate_z(corner1_x,corner1_y,w_z);
  if(corner1_z.first==false) return false;

  float corner2_x;float corner2_y;std::pair<bool,float> corner2_z;
  corner2_x = w_x + (-1.0)*car_size_x_2*std::cos(w_yaw) - car_size_y_2*std::sin(w_yaw);
  corner2_y = w_y + (-1.0)*car_size_x_2*std::sin(w_yaw) + car_size_y_2*std::cos(w_yaw);
  corner2_z = Caculate_z(corner2_x,corner2_y,w_z);
  if(corner2_z.first==false) return false;

  float corner3_x;float corner3_y;std::pair<bool,float> corner3_z;
  corner3_x = w_x + car_size_x_2*std::cos(w_yaw) - (-1.0)*car_size_y_2*std::sin(w_yaw);
  corner3_y = w_y + car_size_x_2*std::sin(w_yaw) + (-1.0)*car_size_y_2*std::cos(w_yaw);
  corner3_z = Caculate_z(corner3_x,corner3_y,w_z);
  if(corner3_z.first==false) return false;

  float corner4_x;float corner4_y;std::pair<bool,float> corner4_z;
  corner4_x = w_x + (-1.0)*car_size_x_2*std::cos(w_yaw) - (-1.0)*car_size_y_2*std::sin(w_yaw);
  corner4_y = w_y + (-1.0)*car_size_x_2*std::sin(w_yaw) + (-1.0)*car_size_y_2*std::cos(w_yaw);
  corner4_z = Caculate_z(corner4_x,corner4_y,w_z);
  if(corner4_z.first==false) return false;

  float v1[3] = {corner1_x-corner2_x,corner1_y-corner2_y,corner1_z.second-corner2_z.second};
  float v2[3] = {corner1_x-corner3_x,corner1_y-corner3_y,corner1_z.second-corner3_z.second};
  float v_verti[3] = {v1[1]*v2[2]-v1[2]*v2[1],v1[2]*v2[0]-v1[0]*v2[2],v1[0]*v2[1]-v1[1]*v2[0]};  
  float p_d = v_verti[0] * corner4_x + v_verti[1] * corner4_y + v_verti[2] * corner4_z.second;
  p_d = 0-p_d;
  float dist_error = std::abs(v_verti[0]*corner4_x + v_verti[1]*corner4_y + v_verti[2]*corner4_z.second + p_d) / 
  std::sqrt(v_verti[0]*v_verti[0] + v_verti[1]*v_verti[1] + v_verti[2]*v_verti[2]); 
  if(dist_error>fourwheel_error)return false;
  return true;
}