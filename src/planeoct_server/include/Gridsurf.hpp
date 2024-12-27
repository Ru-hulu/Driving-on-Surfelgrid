#include<vector>
#include<unordered_map>
#include<map>
#include<iostream>
#include"Surf.hpp"
//基本功能
//1、能够将坐标转换为Key
//2、能够将Key转换为坐标()
//3、采用拉链法解决冲突


// static const nState NOTUS = -2;
// static const nState UNTRA = -1;
// static const nState FRONT =  0;
// static const nState TRAVE =  1;


struct pos_xy
{
    int x;
    int y;
    pos_xy(int x_,int y_):x(x_),y(y_){};
};

struct pos_xyz
{
    int x;
    int y;
    int z;
    pos_xyz(int x_,int y_,int z_):x(x_),y(y_),z(z_){};
};

struct CompareKeyGrid
{
    bool operator()(pos_xy pos_a,pos_xy pos_b)
    {
        if(pos_a.x<pos_b.x)return true;
        else if(pos_a.x == pos_b.x && pos_a.y < pos_b.y) return true;
        else return false;
    }
};
class GridSurf
{
    public:
        GridSurf(float map_resolution,int initial_width_x,int initial_heigh_y);
        //往内部添加元素。
        Surfel* AddSurf(int key_x,int key_y,int status,float a,float b,float c,float d,float p_inline,float abs_dis);
        //给一个空间坐标判定这个surfel是不是已经存在了
        Surfel* SurfelCheck(float pos_w_x,float pos_w_y,float pos_w_z);
        void Pos2Key(float pos_w_x,float pos_w_y,int& key_x,int& key_y);
        void Key2Pos(int key_x,int key_y,float& pos_w_x,float& pos_w_y);
        Surfel* find(int key_x,int key_y);
        inline void add(int key_x,int key_y,Surfel* this_s);
        Surfel*** MultiLayer_Grid;
        std::vector<std::pair<int,int>> exp_grid;//存放的是世界坐标的key
        int width_x = 0;
        int heigh_y = 0;
        float map_res;
        static const int NOTUS = -2;
        static const int UNTRA = -1;
        static const int FRONT =  0;
        static const int TRAVE =  1;
};