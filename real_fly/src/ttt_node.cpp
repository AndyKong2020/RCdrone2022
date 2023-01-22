#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <real_fly/cv.h>


typedef struct {
    double x;
    double y;
    double z;
}zuobiao;

class flyer_type{
public:
    uint8_t state;
    zuobiao lujing[20]={
            {0,0,0.8},
        {0.5,1,0.8},
        {1.4,1.0,0.8},
        {1.9,1.0,0.8},
            {2.20,0.93,0.8},
        {2.5,1.35,0.8},
        {3.2,1.35,0.8},
        {4.2,1.0,0.8},
            {4.72,0.83,0.8},
        {4.6,1.29,0.8},
        {4.6,-1,64.8},
        {4.6,-1.9,0.8},
            {4.90,-2.18,0.8},
        {4.0,-2,0.8},
        {3.5,-1.7,0.8},
        {3.0,-1.5,0.8},
            {2.70,-1.52,0.8},
            {1.59,-2.0,1},
        {-0.17,-2.0,1},
        {0.17,-2.0,-0.5}

    };
    geometry_msgs::PoseStamped pose;


int pose_compara(const zuobiao& t,const geometry_msgs::PoseStamped& pose);
};

int flyer_type::pose_compara(const zuobiao& t,const geometry_msgs::PoseStamped& pose)
{

    double s[3];
    s[0] = t.x - pose.pose.position.x;
    s[1] = t.y - pose.pose.position.y;
    s[2] = t.z - pose.pose.position.z;
    ROS_INFO("chaju:%f,%f,%f",s[0],s[1],s[2]);
    if(s[0]>-0.05 && s[0]<0.05 && s[1]<0.05 && s[1]>-0.05 && s[2]>-0.05 && s[2]<0.05)
    {
        return 0;
    }

    return 1;

}
flyer_type flyer;

void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& pmsg)
{
    flyer.pose = *pmsg;
}

int main(int argc,char* argv[])
{
    ros::init(argc,argv,"ttt_node");
    ros::NodeHandle nh;
    ros::Publisher pose_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local",10);
    ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose",10,pose_cb);
    ros::ServiceClient cv_client = nh.serviceClient<real_fly::cv>("real_fly/cv");
    geometry_msgs::PoseStamped pose;

    real_fly::cv cv;
    cv_client.waitForExistence();

    flyer.state=0;
    ros::Rate rate(20);
    uint8_t check_put=1;//0是yolo检测,1是快递投放，3是重新定高
    while(ros::ok())
    {
        switch(flyer.state)
        {
            case 0:

                pose.pose.position.x = flyer.lujing[0].x;
                pose.pose.position.y = flyer.lujing[0].y;
                pose.pose.position.z = flyer.lujing[0].z;
                if(!flyer.pose_compara(flyer.lujing[0],flyer.pose))
                {
                    flyer.state = 1;
                }
                break;

            case 1:
                pose.pose.position.x = flyer.lujing[1].x;
                pose.pose.position.y = flyer.lujing[1].y;
                pose.pose.position.z = flyer.lujing[1].z;
                if(!flyer.pose_compara(flyer.lujing[1],flyer.pose))
                {
                    flyer.state = 2;
                }
                break;
            case 2:
                pose.pose.position.x = flyer.lujing[2].x;
                pose.pose.position.y = flyer.lujing[2].y;
                pose.pose.position.z = flyer.lujing[2].z;
                if(!flyer.pose_compara(flyer.lujing[2],flyer.pose))
                {
                    flyer.state = 3;
                }
                break;
            case 3:
                pose.pose.position.x = flyer.lujing[3].x;
                pose.pose.position.y = flyer.lujing[3].y;
                pose.pose.position.z = flyer.lujing[3].z;
                if(!flyer.pose_compara(flyer.lujing[3],flyer.pose))
                {
                    flyer.state = 4;
                }
                break;
            case 4:
                pose.pose.position.x = flyer.lujing[4].x;
                pose.pose.position.y = flyer.lujing[4].y;
                pose.pose.position.z = flyer.lujing[4].z;
                if(!flyer.pose_compara(flyer.lujing[4],flyer.pose))
                {

                    if(check_put == 1)
                    {
                        pose.pose.position.z = flyer.lujing[4].z = 0.2;
                        if(flyer.pose.pose.position.z - pose.pose.position.z < 0.015 && flyer.pose.pose.position.z - pose.pose.position.z > -0.015)//在目标点内
                        {
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
                        pose.pose.position.z = flyer.lujing[4].z =0.8 ;
                        if(flyer.pose.pose.position.z - pose.pose.position.z < 0.01 && flyer.pose.pose.position.z - pose.pose.position.z > -0.01)//在目标点内
                        {
                            check_put = 1;
                            flyer.state = 5;
                            
                        }

                    }

                }

                break;
            case 5:
                pose.pose.position.x = flyer.lujing[5].x;
                pose.pose.position.y = flyer.lujing[5].y;
                pose.pose.position.z = flyer.lujing[5].z;
                if(!flyer.pose_compara(flyer.lujing[5],flyer.pose))
                {
                    flyer.state = 6;
                }
                break;
            case 6:
                pose.pose.position.x = flyer.lujing[6].x;
                pose.pose.position.y = flyer.lujing[6].y;
                pose.pose.position.z = flyer.lujing[6].z;
                if(!flyer.pose_compara(flyer.lujing[6],flyer.pose))
                {
                    flyer.state = 7;
                }
                break;
            case 7:
                pose.pose.position.x = flyer.lujing[7].x;
                pose.pose.position.y = flyer.lujing[7].y;
                pose.pose.position.z = flyer.lujing[7].z;
                if(!flyer.pose_compara(flyer.lujing[7],flyer.pose))
                {
                    flyer.state = 8;
                }
                break;
            case 8:
                pose.pose.position.x = flyer.lujing[8].x;
                pose.pose.position.y = flyer.lujing[8].y;
                pose.pose.position.z = flyer.lujing[8].z;
                if(!flyer.pose_compara(flyer.lujing[8],flyer.pose))
                {
                    if(check_put == 1)
                    {
                        pose.pose.position.z = flyer.lujing[8].z = 0.2;
                        if(flyer.pose.pose.position.z - pose.pose.position.z < 0.02 && flyer.pose.pose.position.z - pose.pose.position.z > -0.02)//在目标点内
                        {
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
                        pose.pose.position.z = flyer.lujing[8].z =0.8 ;
                        if(flyer.pose.pose.position.z - pose.pose.position.z < 0.02 && flyer.pose.pose.position.z - pose.pose.position.z > -0.02)//在目标点内
                        {
                            check_put = 1;
                            flyer.state = 9;
                            
                        }

                    }
                }
                break;
            case 9:
                pose.pose.position.x = flyer.lujing[9].x;
                pose.pose.position.y = flyer.lujing[9].y;
                pose.pose.position.z = flyer.lujing[9].z;
                if(!flyer.pose_compara(flyer.lujing[9],flyer.pose))
                {
                    flyer.state = 10;
                }
                break;
            case 10:
                pose.pose.position.x = flyer.lujing[10].x;
                pose.pose.position.y = flyer.lujing[10].y;
                pose.pose.position.z = flyer.lujing[10].z;
                if(!flyer.pose_compara(flyer.lujing[10],flyer.pose))
                {
                    flyer.state = 11;
                }
                break;
            case 11:
                pose.pose.position.x = flyer.lujing[11].x;
                pose.pose.position.y = flyer.lujing[11].y;
                pose.pose.position.z = flyer.lujing[11].z;
                if(!flyer.pose_compara(flyer.lujing[11],flyer.pose))
                {
                    flyer.state = 12;
                }
                break;
            case 12:
                pose.pose.position.x = flyer.lujing[12].x;
                pose.pose.position.y = flyer.lujing[12].y;
                pose.pose.position.z = flyer.lujing[12].z;
                if(!flyer.pose_compara(flyer.lujing[12],flyer.pose))
                {
                    // if(check_put == 1)
                    // {
                    //     pose.pose.position.z = flyer.lujing[12].z = 0.2;
                    //     if(flyer.pose.pose.position.z - pose.pose.position.z < 0.02 && flyer.pose.pose.position.z - pose.pose.position.z > -0.02)//在目标点内
                    //     {
                    //         cv.request.state = 2;
                    //         bool flag = cv_client.call(cv);
                    //         if(flag == true)
                    //         {
                    //             ros::Duration(4);
                    //             check_put = 2;
                    //         }
                    //     }


                    // }
                    // else if(check_put == 2)
                    // {
                    //     pose.pose.position.z = flyer.lujing[12].z =0.8 ;
                    //     if(flyer.pose.pose.position.z - pose.pose.position.z < 0.02 && flyer.pose.pose.position.z - pose.pose.position.z > -0.02)//在目标点内
                    //     {
                    //         check_put = 1;
                    //         flyer.state = 13;
                            
                    //     }

                    // }
                    flyer.state = 13;
                }
                break;
            case 13:
                pose.pose.position.x = flyer.lujing[13].x;
                pose.pose.position.y = flyer.lujing[13].y;
                pose.pose.position.z = flyer.lujing[13].z;
                if(!flyer.pose_compara(flyer.lujing[13],flyer.pose))
                {
                    flyer.state = 14;
                }
                break;
            case 14:
                pose.pose.position.x = flyer.lujing[14].x;
                pose.pose.position.y = flyer.lujing[14].y;
                pose.pose.position.z = flyer.lujing[14].z;
                if(!flyer.pose_compara(flyer.lujing[14],flyer.pose))
                {
                    flyer.state = 15;
                }
                break;
            case 15:
                pose.pose.position.x = flyer.lujing[15].x;
                pose.pose.position.y = flyer.lujing[15].y;
                pose.pose.position.z = flyer.lujing[15].z;
                if(!flyer.pose_compara(flyer.lujing[15],flyer.pose))
                {
                    flyer.state = 16;
                }
                break;
            case 16:
                pose.pose.position.x = flyer.lujing[16].x;
                pose.pose.position.y = flyer.lujing[16].y;
                pose.pose.position.z = flyer.lujing[16].z;
                if(!flyer.pose_compara(flyer.lujing[16],flyer.pose))
                {
                    if(check_put == 1)
                    {
                        pose.pose.position.z = flyer.lujing[16].z = 0.2;
                        if(flyer.pose.pose.position.z - pose.pose.position.z < 0.02 && flyer.pose.pose.position.z - pose.pose.position.z > -0.02)//在目标点内
                        {
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
                        pose.pose.position.z = flyer.lujing[16].z =0.8 ;
                        if(flyer.pose.pose.position.z - pose.pose.position.z < 0.02 && flyer.pose.pose.position.z - pose.pose.position.z > -0.02)//在目标点内
                        {
                            check_put = 1;
                            flyer.state = 17;
                            
                        }

                    }

                }
                
                break;
            case 17:
                pose.pose.position.x = flyer.lujing[17].x;
                pose.pose.position.y = flyer.lujing[17].y;
                pose.pose.position.z = flyer.lujing[17].z;
                if(!flyer.pose_compara(flyer.lujing[17],flyer.pose))
                {
                    flyer.state = 18;
                }
                break;
            case 18:
                pose.pose.position.x = flyer.lujing[18].x;
                pose.pose.position.y = flyer.lujing[18].y;
                pose.pose.position.z = flyer.lujing[18].z;
                if(!flyer.pose_compara(flyer.lujing[18],flyer.pose))
                {
                    flyer.state = 19;
                }
                break;
            case 19:
                pose.pose.position.x = flyer.lujing[19].x;
                pose.pose.position.y = flyer.lujing[19].y;
                pose.pose.position.z = flyer.lujing[19].z;
                break;

        }

        ROS_INFO("pose:%f,%f,%f",pose.pose.position.x,pose.pose.position.y,pose.pose.position.z);
        pose_pub.publish(pose);

        ros::spinOnce();
        rate.sleep();


    }


}