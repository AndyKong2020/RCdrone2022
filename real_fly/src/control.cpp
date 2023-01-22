#include <real_fly/control.h>


int flyer_type::pose_compara(zuobiao _zuobiao,geometry_msgs::PoseStamped _pose)
{
    // if(ros::Time::now().toSec() - _pose.header.stamp.toSec() > 20);
    // {
    //     return 1;
    // }

    ROS_INFO("%f %f\n",_pose.pose.position.x - _zuobiao.x,_pose.pose.position.y - _zuobiao.y);

    if(_pose.pose.position.x - _zuobiao.x >-0.02 && _pose.pose.position.x - _zuobiao.x <0.02
    &&_pose.pose.position.y - _zuobiao.y >-0.02 && _pose.pose.position.y - _zuobiao.y <0.02)//再目标点范围内
    {

        return 0;
    }

    return 2;

}
void flyer_type::zuobiao_update(zuobiao& _zuobiao,zuobiao& pianyi)
{
    _zuobiao.x =_zuobiao.x + PX * pianyi.x;
    _zuobiao.y =_zuobiao.y + PY * pianyi.y;
    /*缺一个修正有效性检测：修正目标不能在树的附近*/
}

