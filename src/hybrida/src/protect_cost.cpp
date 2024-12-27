



double Algorithm::costFunction(const std::vector<double> &x, std::vector<double>& grad ,void* func_data)
{
    float p_index[3]     ={0,0,0};//当前节点的世界坐标
    int x_index = 0;//被优化参数的索引
    Algorithm* obj_self = reinterpret_cast<Algorithm*>(func_data);
    int point_number = obj_self->une_tra_data.size();//轨迹长度
    int now_point_index = x_index/3;
    double loss_this_iteration = 0.0;
    obj_self->opt_cct++;

    for(int i=0;i<grad.size();i++)grad[i] = 0.0;
    while(x_index<x.size())
    {
        now_point_index = x_index/3;
        p_index[0] = float(x[x_index]);
        p_index[1] = float(x[x_index+1]);
        p_index[2] = float(x[x_index+2]);
 
        int useless_z = 0;
        float pro_a,pro_b,pro_c,pro_d;
        pro_a = obj_self->path_local_geometry[now_point_index]->a;
        pro_b = obj_self->path_local_geometry[now_point_index]->b;
        pro_c = obj_self->path_local_geometry[now_point_index]->c;
        pro_d = obj_self->path_local_geometry[now_point_index]->d;
 
        Eigen::Vector4f car_center = {p_index[0],p_index[1],p_index[2],1};
        Eigen::Vector4f org_center = {0,0,0,1};
        float dis_nor = (car_center-org_center).norm();

        std::vector<Eigen::Vector3f> local_constraint = obj_self->une_tra_constraint[now_point_index];
        Eigen::Matrix4f Twp = obj_self->une_tra_planes_Twp[now_point_index];
        car_center = obj_self->une_tra_planes_Tpw[now_point_index]*car_center;
        // f = (x-p)^2

        //     grad[0] = 2*(car_center-org_center)[0];
        //     grad[1] = 2*(car_center-org_center)[1];
        //     grad[2] = 2*(car_center-org_center)[2];

        // loss_this_iteration = dis_nor;
        // std::cout<<"corner_p "<<corner_p<<std::endl;
        //车身坐标在局部坐标系下的表示,z=0
        //在局部坐标系下计算constraint带来的梯度
        // if(0)
        for(int c = 0;c<local_constraint.size();c++)
        {
            float tem_loss = 0.0;
            Eigen::Vector2f local_grad_x_y;
            Eigen::Vector4f local_grad;
            Eigen::Vector4f local_org={0,0,0,1};

            Eigen::Vector4f word_grad;
            Cost_obs({car_center(0),car_center(1)},local_constraint[c],obj_self->obs_safty_th,tem_loss,local_grad_x_y);
            loss_this_iteration += tem_loss;
            local_grad(0) = local_grad_x_y(0);
            local_grad(1) = local_grad_x_y(1);
            local_grad(2) = 0;//在局部坐标系下,梯度的表示
            local_grad(3) = 1;
            // if(std::abs(local_grad(0))>=1e-3||std::abs(local_grad(1))>=1e-3)
            // std::cout<<"car "<<x_index/3<<" corner"<<i<<std::endl;
            word_grad = Twp * (local_grad-local_org);
            grad[x_index]   += word_grad(0)*(-1.0);
            grad[x_index+1] += word_grad(1)*(-1.0);
            grad[x_index+2] += word_grad(2)*(-1.0);   
        }   //将梯度转换到世界坐标系下
        // std::cout<<"car "<<x_index/3<<" grad "
        // <<grad[x_index]  <<" "
        // <<grad[x_index+1]<<" "
        // <<grad[x_index+2]<<" "<<std::endl;
        x_index+=3;
    }
 
    std::cout<<"opt_cct: "<<obj_self->opt_cct<<"loss_this_iteration "<<loss_this_iteration<<std::endl;

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
