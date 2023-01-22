#include <ros/ros.h>
#include <real_fly/control.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <real_fly/yolo.h>
#include <real_fly/cv.h>

float Gao = 0;
flyer_type flyer;

void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& pmsg)
{
    flyer.pose = *pmsg;
    // ROS_INFO("x=%f,y=%f,z=%f",flyer.pose.pose.position.x,
    // flyer.pose.pose.position.y,
    // flyer.pose.pose.position.z
    // );
}

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& pmsg)
{
    current_state = *pmsg;
}

int main(int argc,char* argv[])
{
    ros::init(argc,argv,"flyer");
    ros::NodeHandle nh;

    /*无人机位置和速度发布者*/
    ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("mavros/setpoint_velocity/cmd_vel_unstamped",10);
    ros::Publisher pose_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local",10);

    /*订阅无人机位置、模式*/
    ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose",10,pose_cb);
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state",10,state_cb);

    /*无人机模式设置服务通信*/
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
    ros::ServiceClient yolo_client = nh.serviceClient<real_fly::yolo>("real_fly/yolo");
    ros::ServiceClient cv_client = nh.serviceClient<real_fly::cv>("real_fly/cv");
    ros::Rate rate(30);

    geometry_msgs::PoseStamped pose;
    geometry_msgs::Twist twi;

    real_fly::cv cv;
    cv_client.waitForExistence();

    // real_fly::yolo yolo;
    // yolo_client.waitForExistence();
    // while(ros::ok())
    // {
    //     yolo.request.fa = 1;
    //     yolo_client.call(yolo);
    //     ROS_INFO("%d %d",yolo.response.x,yolo.response.y);
    //}



    ros::Time last_request = ros::Time::now();

    flyer.state = 0;

    uint8_t check_put=1;//0是yolo检测,1是快递投放，3是重新定高

    while(ros::ok())
    {

        switch (flyer.state)
        {
        case 0:
        /*起飞，定高1m，速度设置0.1m/s*/
            pose.header.stamp = ros::Time::now();
            pose.pose.position.x = 0; 
            pose.pose.position.y = 0;
            pose.pose.position.z = 1.8; 

            /*到一定高度后切换状态*/
            if(flyer.pose.pose.position.z - pose.pose.position.z < 0.015 && flyer.pose.pose.position.z - pose.pose.position.z > -0.015)//在目标点内
            {
                flyer.state = 6;
            }

            break;

        case 1://投放目标1
        /*先飞到目标点附近*/
            pose.header.stamp = ros::Time::now();
            pose.pose.position.x = flyer.toufang[0].x;
            pose.pose.position.y = flyer.toufang[0].y;
            pose.pose.position.z = flyer.toufang[0].z;

            if(!flyer.pose_compara(flyer.toufang[0],flyer.pose))
            {   
                if(check_put == 1)
                {
                    if(flyer.pose.pose.position.z - pose.pose.position.z < 0.015 && flyer.pose.pose.position.z - pose.pose.position.z > -0.015)//在目标点内
                    {
                        /*缓慢到目标点*/
                        if(flyer.toufang[0].z >= 0.5)
                        {
                            pose.pose.position.z = flyer.toufang[0].z = flyer.toufang[0].z-0.5;
                        }

                        cv.request.state = 1;
                        bool flag = cv_client.call(cv);
                        if(flag == true)
                        {
                            ros::Duration(4);
                            check_put = 2;
                        }
                    }


                }
                else if(check_put == 2)
                {
                    pose.pose.position.z = flyer.toufang[0].z =1.8 ;
                    if(flyer.pose.pose.position.z - pose.pose.position.z < 0.01 && flyer.pose.pose.position.z - pose.pose.position.z > -0.01)//在目标点内
                    {
                        check_put = 1;
                        flyer.state = 2;
                        
                    }

                }

            }

        /*飞到目标点后，进行yolo检测，修正目标点*/
            // if(flyer.pose_compara(flyer.toufang[0],flyer.pose) == 0)
            // {
                
            //     /*yolo检测*/
            //     if(check_put == 0)
            //     {
            //         yolo.request.fa=1;
            //         bool flag = yolo_client.call(yolo);
            //         flyer.yolo_pianyi.x = yolo.response.x - flyer.yolo_conter.x;
            //         flyer.yolo_pianyi.y = yolo.response.y - flyer.yolo_conter.y;

            //         flyer.zuobiao_update(flyer.toufang[0],flyer.yolo_pianyi);

            //         /*当yolo检测后的偏移量很小，进行投放*/
            //         if(flyer.yolo_pianyi.x == 0 && flyer.yolo_pianyi.y == 0)
            //         {
            //             check_put = 1;
            //         }
            //     }
            //     else if(check_put == 1)
            //     {
            //         pose.pose.position.z = 0.5;

            //         if(flyer.pose.pose.position.z - pose.pose.position.z < 0.01 && flyer.pose.pose.position.z - pose.pose.position.z > -0.01)//在目标点内
            //         {
            //             uint8_t send_buf[7] = "Servo0";
            //             send_buf[5] = 1 + 48;
            //             flyer.sp.write(send_buf,7);   
            //         }
            //         /*等待接受完成指令*/
            //         uint8_t buffer[5];
            //         flyer.sp.read(buffer,4);
            //         /*串口数据处理*/
            //         Is_Servo_Request(buffer,5);
            //         if(0)
            //         {
            //             check_put = 3;
            //         }
            //     }
            //     else if(check_put == 3)
            //     {
            //         if(!flyer.pose_compara(flyer.toufang[0],flyer.pose))
            //         {
            //             check_put = 0;
            //             flyer.state = 2;
            //         }
            //     }
                
                
            // }
            break;

        case 2://投放目标2
            pose.header.stamp = ros::Time::now();
            pose.pose.position.x = flyer.toufang[1].x;
            pose.pose.position.y = flyer.toufang[1].y;
            pose.pose.position.z = flyer.toufang[1].z;

            if(!flyer.pose_compara(flyer.toufang[1],flyer.pose))
            {   
                if(check_put == 1)
                {
                    if(flyer.pose.pose.position.z - pose.pose.position.z < 0.015 && flyer.pose.pose.position.z - pose.pose.position.z > -0.015)//在目标点内
                    {
                        /*缓慢到目标点*/
                        if(flyer.toufang[1].z >= 0.5)
                        {
                            pose.pose.position.z = flyer.toufang[1].z = flyer.toufang[1].z-0.5;
                        }

                        cv.request.state = 2;
                        bool flag = cv_client.call(cv);
                        if(flag == true)
                        {
                            ros::Duration(4);
                            check_put = 2;
                        }
                    }


                }
                else if(check_put == 2)
                {
                    pose.pose.position.z = flyer.toufang[1].z =1.8 ;
                    if(flyer.pose.pose.position.z - pose.pose.position.z < 0.015 && flyer.pose.pose.position.z - pose.pose.position.z > -0.015)//在目标点内
                    {
                        check_put = 1;
                        flyer.state = 3;
                        
                    }

                }
            }
            break;

        case 3://投放目标3
            pose.header.stamp = ros::Time::now();
            pose.pose.position.x = flyer.toufang[2].x;
            pose.pose.position.y = flyer.toufang[2].y;
            pose.pose.position.z = flyer.toufang[2].z;

            if(!flyer.pose_compara(flyer.toufang[2],flyer.pose))
            {   
                if(check_put == 1)
                {
                    if(flyer.pose.pose.position.z - pose.pose.position.z < 0.015 && flyer.pose.pose.position.z - pose.pose.position.z > -0.015)//在目标点内
                    {
                        /*缓慢到目标点*/
                        if(flyer.toufang[2].z >= 0.5)
                        {
                            pose.pose.position.z = flyer.toufang[2].z = flyer.toufang[2].z-0.5;
                        }

                        cv.request.state = 3;
                        bool flag = cv_client.call(cv);
                        if(flag == true)
                        {
                            ros::Duration(4);
                            check_put = 2;
                        }
                    }


                }
                else if(check_put == 2)
                {
                    pose.pose.position.z = flyer.toufang[2].z =1.8 ;
                    if(flyer.pose.pose.position.z - pose.pose.position.z < 0.015 && flyer.pose.pose.position.z - pose.pose.position.z > -0.015)//在目标点内
                    {
                        check_put = 1;
                        flyer.state = 4;
                        
                    }

                }
            break;

        case 4://门
        /*先飞到目标点附近*/
            flyer.door1.x = flyer.door.x + 1.5;
            flyer.door1.y = flyer.door.y;
            flyer.door1.z = flyer.door.z;

            if(flyer.door1.x - flyer.pose.pose.position.x < 0.015 && flyer.door1.x - flyer.pose.pose.position.x > -0.015
            && flyer.door1.y - flyer.pose.pose.position.y < 0.015 && flyer.door1.y - flyer.pose.pose.position.y > -0.015
            && flyer.door1.z - flyer.pose.pose.position.z < 0.015 && flyer.door1.z - flyer.pose.pose.position.z > -0.015
            )
            {
                flyer.door1.x = flyer.door.x;
            }


            break;

        case 5:
            pose.header.stamp = ros::Time::now();
            pose.pose.position.x = 0; 
            pose.pose.position.y = 0;
            pose.pose.position.z = 0.2; 

            /*到一定高度后切换状态*/
            if(flyer.pose.pose.position.z - pose.pose.position.z < 0.015 && flyer.pose.pose.position.z - pose.pose.position.z > -0.015)//在目标点内
            {
                pose.pose.position.z =pose.pose.position.z - 0.08;
            }
            break;

    /*以下是巡航*/
        case 6:
            pose.pose.position.x = flyer.renwu[0].x;
            pose.pose.position.y = flyer.renwu[0].y;
            pose.pose.position.z = flyer.renwu[0].z;
            if(!flyer.pose_compara(flyer.renwu[0],flyer.pose))
            {
                ros::Duration(2);

                flyer.state = 7;
            }
            break;
        case 7:
            pose.pose.position.x = flyer.renwu[1].x;
            pose.pose.position.y = flyer.renwu[1].y;
            pose.pose.position.z = flyer.renwu[1].z;
            if(!flyer.pose_compara(flyer.renwu[1],flyer.pose))
            {
                ros::Duration(2);

                flyer.state = 8;
            }
            break;
        case 8:
            pose.pose.position.x = flyer.renwu[2].x;
            pose.pose.position.y = flyer.renwu[2].y;
            pose.pose.position.z = flyer.renwu[2].z;
            if(!flyer.pose_compara(flyer.renwu[2],flyer.pose))
            {
                ros::Duration(2);

                flyer.state = 9;
            }
            break;
        case 9:
            pose.pose.position.x = flyer.renwu[3].x;
            pose.pose.position.y = flyer.renwu[3].y;
            pose.pose.position.z = flyer.renwu[3].z;
            if(!flyer.pose_compara(flyer.renwu[3],flyer.pose))
            {
                ros::Duration(2);

                flyer.state = 10;
            }
            break;
        case 10:
            pose.pose.position.x = flyer.renwu[4].x;
            pose.pose.position.y = flyer.renwu[4].y;
            pose.pose.position.z = flyer.renwu[4].z;
            if(!flyer.pose_compara(flyer.renwu[4],flyer.pose))
            {
                ros::Duration(2);

                flyer.state = 11;
            }
            break;
        case 11:
            pose.pose.position.x = flyer.renwu[5].x;
            pose.pose.position.y = flyer.renwu[5].y;
            pose.pose.position.z = flyer.renwu[5].z;
            if(!flyer.pose_compara(flyer.renwu[5],flyer.pose))
            {
                ros::Duration(2);

                flyer.state = 12;
            }
            break;
        case 12:
            pose.pose.position.x = flyer.renwu[6].x;
            pose.pose.position.y = flyer.renwu[6].y;
            pose.pose.position.z = flyer.renwu[6].z;
            if(!flyer.pose_compara(flyer.renwu[6],flyer.pose))
            {
                ros::Duration(2);

                flyer.state = 13;
            }
            break;
        case 13:
            pose.pose.position.x = flyer.renwu[7].x;
            pose.pose.position.y = flyer.renwu[7].y;
            pose.pose.position.z = flyer.renwu[7].z;
            if(!flyer.pose_compara(flyer.renwu[7],flyer.pose))
            {
                ros::Duration(2);

                flyer.state = 14;
            }
            break;
        case 14:
            pose.pose.position.x = flyer.renwu[8].x;
            pose.pose.position.y = flyer.renwu[8].y;
            pose.pose.position.z = flyer.renwu[8].z;
            if(!flyer.pose_compara(flyer.renwu[8],flyer.pose))
            {
                ros::Duration(2);

                flyer.state = 15;
            }
            break;
        case 15:
            pose.pose.position.x = flyer.renwu[9].x;
            pose.pose.position.y = flyer.renwu[9].y;
            pose.pose.position.z = flyer.renwu[9].z;
            if(!flyer.pose_compara(flyer.renwu[9],flyer.pose))
            {
                ros::Duration(2);

                flyer.state = 16;
            }
            break;
        case 16:
            pose.pose.position.x = flyer.renwu[10].x;
            pose.pose.position.y = flyer.renwu[10].y;
            pose.pose.position.z = flyer.renwu[10].z;
            if(!flyer.pose_compara(flyer.renwu[10],flyer.pose))
            {
                ros::Duration(2);

                flyer.state = 17;
            }
            break;
        case 17:
            pose.pose.position.x = flyer.renwu[11].x;
            pose.pose.position.y = flyer.renwu[11].y;
            pose.pose.position.z = flyer.renwu[11].z;
            if(!flyer.pose_compara(flyer.renwu[11],flyer.pose))
            {
                ros::Duration(2);

                flyer.state = 18;
            }
            break;
        case 18:
            pose.pose.position.x = flyer.renwu[12].x;
            pose.pose.position.y = flyer.renwu[12].y;
            pose.pose.position.z = flyer.renwu[12].z;
            if(!flyer.pose_compara(flyer.renwu[12],flyer.pose))
            {
                ros::Duration(2);

                flyer.state = 19;
            }
            break;
        case 19:
            pose.pose.position.x = flyer.renwu[13].x;
            pose.pose.position.y = flyer.renwu[13].y;
            pose.pose.position.z = flyer.renwu[13].z;
            if(!flyer.pose_compara(flyer.renwu[13],flyer.pose))
            {
                ros::Duration(2);

                flyer.state = 1;
            }
            break;
        }

        ROS_INFO("%f,%f,%f",pose.pose.position.x,pose.pose.position.y,pose.pose.position.z);
        pose_pub.publish(pose);
        // vel_pub.publish(twi);

        ros::spinOnce();
        rate.sleep();
    }
    }
}

