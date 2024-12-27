#include<iostream>
struct Surfel
{
    float a;
    float b;
    float c;
    float d;
    float typical_z;
    float p_inline;
    float abs_dis;
    int status;
    Surfel* next_level;
    Surfel():next_level(NULL),status(-2){};
};//44个字节