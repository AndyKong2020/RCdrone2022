#ifndef control_h
#define control_h

#include <ros/ros.h>
#include <mavros_msgs/State.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <serial/serial.h>

typedef struct {
    double x;
    double y;
    double z;
}zuobiao;

class flyer_type{
public:
    uint8_t state;//无人机飞行过程控制

    zuobiao renwu[14]={{},{},{},{},{},{},{},{},{},{},{},{},{},{}};

    zuobiao toufang[3]={{2,0,2},{1,1,1},{0,1,1}};//三个投放位置

    zuobiao door={0,0,1.0};//门的位置
    zuobiao door1;

    zuobiao tree[2];//树的位置

    zuobiao land;

    zuobiao yolo_conter;//无人机在图像中的等价坐标

    zuobiao yolo_pianyi;//目标在图像中的坐标偏移

    float PX,PY;//偏移修正比例系数

    geometry_msgs::PoseStamped pose;//无人机的位置

    serial::Serial sp;

    // geometry_msgs::Twist twi;
int pose_compara(zuobiao _zuobiao,geometry_msgs::PoseStamped _pose);//对比是否到达目标点

void zuobiao_update(zuobiao& _zuobiao,zuobiao& pianyi);//根据yolo偏移修正坐标位置

};

#endif