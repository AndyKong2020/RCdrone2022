#include <ros/ros.h>
#include <serial/serial.h>
#include <real_fly/cv.h>

int door_offset(uint8_t *uart_data, size_t n)
{
    int result = 0;
    if(n == 5 && uart_data[0] == '1')
    {
        result = (uart_data[1] - 48) * 100 + (uart_data[2] - 48) * 10 + uart_data[1] - 48 - 100;
    }
    return result;
}

void servo_send(uint8_t state)  //舵机
{
    uint8_t send_buf[7] = "Servo0";
    send_buf[5] = state + 48;
}

int Is_Servo_Request(uint8_t uart_data[], size_t n)
{
    if(n == 5)
    {
        if(!strcmp((const char *)uart_data, "Serv"))
        {
            return 1;
        }
        else
        {
            return 0;
        }
    }
    else
    {
        return 0;
    }
}

bool cv_cb(real_fly::cv::Request& req,real_fly::cv::Response& resp)
{
    ROS_INFO("cv_cb ...");
    serial::Serial sp;
    serial::Timeout to = serial::Timeout::simpleTimeout(50);
    sp.setPort("/dev/ttyUSB0");
    sp.setBaudrate(115200);
    sp.setTimeout(to);
    try
    {
        sp.open();
    }
    catch(const std::exception& e)
    {
        ROS_ERROR_STREAM("Unable to open port.");
        return false;
    }

    if(sp.isOpen())
    {
        ROS_INFO_STREAM("/dev/ttyUSB0 is opened.");
    }
    else
    {
        return false;
    }

    uint8_t send_buf[7] = "Servo0";
    send_buf[5] = req.state + 48;
    ROS_INFO("%s",send_buf);
    sp.write(send_buf,7);
    ros::Duration(1);
    uint8_t buffer[5];
    sp.read(buffer,4);
    ROS_INFO("%s",buffer);
    
    sp.close();
    /*串口数据处理*/
    resp.pianyi = door_offset(buffer,5);
    return true;

}

int main(int argc,char *argv[])
{
    ros::init(argc,argv,"cv_node");
    ros::NodeHandle nh;

    ros::ServiceServer cv_server = nh.advertiseService("real_fly/cv",cv_cb);

    ros::spin();

    return 0;
}