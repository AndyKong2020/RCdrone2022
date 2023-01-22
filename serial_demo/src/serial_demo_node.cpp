#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/UInt8.h>

int door_offset(uint8_t *uart_data, size_t n)
{
    int result = 0;
    if(n == 5 && uart_data[0] == '1')
    {
        result = (uart_data[1] - 48) * 100 + (uart_data[2] - 48) * 10 + uart_data[1] - 48 - 100;
    }
    return result;
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


int main(int argc,char *argv[])
{   ros::init(argc,argv,"serial");
    ros::NodeHandle nh;
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
        return -1;
    }

    if(sp.isOpen())
    {
        ROS_INFO_STREAM("/dev/ttyUSB0 is opened.");

    }
    else
    {
        return -1;
    }

    int check_put=1;

    while(ros::ok())
    {
        uint8_t send_buf[7] = "Servo0";
        send_buf[5] = 4 + 48;
        sp.write(send_buf,7);
        ros::Duration(1);
        // ROS_INFO("%s",send_buf); 
        /*等待接受完成指令*/
        uint8_t buffer[5];
        sp.read(buffer,4);
        // ROS_INFO("%s",buffer);
        /*串口数据处理*/
        if(Is_Servo_Request(buffer,5))
        {
            ROS_INFO("%d",door_offset(buffer,5));
        }
    }

    sp.close();

}