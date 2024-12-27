#include"Gridsurf.hpp"
GridSurf::GridSurf(float map_resolution,int initial_width_x,int initial_heigh_y)
{
    map_res = map_resolution;
    width_x = initial_width_x;//默认是400
    heigh_y = initial_heigh_y;//默认是400
    MultiLayer_Grid = new Surfel**[width_x];
    for(int i=0;i<width_x;i++)
    MultiLayer_Grid[i] = new Surfel*[heigh_y];
    for(int i=0;i<width_x;i++)
    for(int j=0;j<heigh_y;j++)
    MultiLayer_Grid[i][j] = nullptr;

    // while(MultiLayer_Grid[161][0]==nullptr)std::cout<<"error";

}

void GridSurf::Pos2Key(float pos_w_x,float pos_w_y,int& key_x,int& key_y)
{
    key_x = pos_w_x/map_res;
    key_y = pos_w_y/map_res;
    if(pos_w_x<0)key_x--;
    if(pos_w_y<0)key_y--;
}
Surfel* GridSurf::find(int key_x,int key_y)
{   //200 200就是 0 0
    if(
      (0<=(width_x/2 + key_x) && (width_x/2 + key_x) <width_x)
    &&(0<=(heigh_y/2 + key_y) && (heigh_y/2 + key_y) <heigh_y)
      )
      {
        return MultiLayer_Grid[width_x/2 + key_x][heigh_y/2 + key_y];
      }
    else
    {
        return nullptr;
    }
}
void GridSurf::add(int key_x,int key_y,Surfel* this_s)
{
    MultiLayer_Grid[width_x/2 + key_x][heigh_y/2 + key_y] = this_s;
}
void GridSurf::Key2Pos(int key_x,int key_y,float& pos_w_x,float& pos_w_y)
{
    pos_w_x = float(key_x) * map_res + 0.5*map_res;
    pos_w_y = float(key_y) * map_res + 0.5*map_res;
}

Surfel* GridSurf::SurfelCheck(float pos_w_x,float pos_w_y,float pos_w_z)
{
    int this_keyx=0;
    int this_keyy=0;
    Pos2Key(pos_w_x,pos_w_y,this_keyx,this_keyy);
    Surfel* this_itt = find(this_keyx,this_keyy);
    if(this_itt==nullptr)return nullptr;
    else//已经能够找到
    {   
        // while(1)
        // std::cout<<"pointer "<<this_itt;
        Surfel* begin_surf = this_itt;
        while(begin_surf)
        {
         float tem_z = begin_surf->typical_z;      
         if(std::abs(tem_z-pos_w_z)<=map_res)return begin_surf;
         else begin_surf = begin_surf->next_level;
        }
        //能够出来就代表没找到
        return nullptr;
    }
}

//这个key_x key_y是世界坐标的Key，取之范围是 -200 200
Surfel* GridSurf::AddSurf(int key_x,int key_y,int status,float a,float b,float c,float d,float p_inline,float abs_dis)
{
        float pos_w_x,pos_w_y,pos_w_z;
        Key2Pos(key_x,key_y,pos_w_x,pos_w_y);
        if(status>=0) pos_w_z = (a * pos_w_x + b * pos_w_y + d)/(0-c);
        else pos_w_z = p_inline;

        Surfel* this_itt = find(key_x,key_y);
        if(this_itt==nullptr)
        {
            this_itt = new Surfel();
            this_itt->a = a;        this_itt->b = b;        this_itt->c = c;        this_itt->d = d;
            this_itt->p_inline = 1; this_itt->abs_dis = abs_dis; this_itt->status = status;//状态已更新
            this_itt->typical_z = pos_w_z; 
            add(key_x,key_y,this_itt);
            exp_grid.push_back({key_x,key_y});
            return this_itt;
        }//这个区块没有被更新过
        else
        {
            Surfel* pre_p = this_itt;
            while(this_itt!=nullptr)
            {
                if(std::abs(this_itt->typical_z - pos_w_z)<=map_res)
                return nullptr;
                else
                {
                    pre_p = this_itt;
                    this_itt = this_itt->next_level;
                }
            }
            Surfel* new_sur = new Surfel();            
            new_sur->a = a;        new_sur->b = b;        new_sur->c = c;        new_sur->d = d;
            new_sur->p_inline = 1; new_sur->abs_dis = abs_dis; new_sur->status = status;//状态已更新
            new_sur->typical_z = pos_w_z;
            pre_p->next_level = new_sur;//已经连接到了链表上
            return new_sur;
        }//这个区块已经是多层区域了
}
//给定一个位置，投影得到