#!/usr/bin/env python
import rospy
import os
from geometry_msgs.msg import Twist
import sys, select, termios, tty
from mavros_msgs.msg import PositionTarget, State ,HomePosition ,Altitude
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import Imu, NavSatFix
from std_msgs.msg import Float32, String, Float32MultiArray,Float64, Int32
import time
import math
import sys
import time

message = """
$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
%%%%%%%%%%%%%%%%%%%%%%%
command_cotrol
%%%%%%%%%%%%%%%%%%%%%%%
1 : TAKEOFF
--------------------------

"""


cur_Position_Target = PositionTarget()
mavros_state = State()
height = Altitude()

armServer = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
setModeServer = rospy.ServiceProxy('/mavros/set_mode', SetMode)
local_target_pub = rospy.Publisher('/mavros/setpoint_raw/local', PositionTarget, queue_size=10)

global cur_Position_Target


vx = 0
vy = 0
vz = 0 



def __init__():
    rospy.init_node('PX4_AuotFLy' ,anonymous=True)
    rospy.Subscriber("/mavros/altitude", Altitude, getheight)
    rospy.Subscriber("/detect", Float32MultiArray, callback)
    rospy.Subscriber("/mavros/local_position/pose", PoseStamped, position_now)
    rospy.Subscriber("/mavros/state", State, mavros_state_callback)
    print("Initialized")


def mavros_state_callback(msg):
    global mavros_state
    mavros_state = msg

def position_now(msg):
    global position
    position = msg
    
    
def Intarget_local(x,y,z):   #,vx,vy,vz,afx,afy,afz

    set_target_local = PositionTarget()
    set_target_local.type_mask = 0b10111111000 #0b10111000000 #0b10111111000
    set_target_local.coordinate_frame = 8 #
    set_target_local.position.x = x
    set_target_local.position.y = y
    set_target_local.position.z = z
    set_target_local.yaw = 1.57
    return set_target_local


def VC():   #,vx,vy,vz,afx,afy,afz
    set_target_local = PositionTarget()
    set_target_local.type_mask = 0b101111000000 #0b10111000111           
    set_target_local.coordinate_frame = 8               
    set_target_local.velocity.x = vx                
    set_target_local.velocity.y = vy
    set_target_local.velocity.z = vz
    set_target_local.yaw = 1.57
    return set_target_local


def callback(msg):
    global cammsg
    cammsg = msg
    
def getheight(msg):
    global height
    height = msg

		
def run_state_update():
    if mavros_state.mode != 'OFFBOARD':
        setModeServer(custom_mode='OFFBOARD')
        local_target_pub.publish(cur_Position_Target)
    else:
        local_target_pub.publish(cur_Position_Target)



#cur_Position_Target = Intarget_local(0,0,2.3)  

def initialize():
    armServer(True)
    setModeServer(custom_mode='OFFBOARD')


#cur_Position_Target = VC()


if __name__=="__main__":
    __init__()
    print(message)
    initialize()
#===========================PART . 1 BEGIN==============================================================================
    #-------------------------------------------------------

#takeoff
    flag = 0
    vx = 0
    vy = 0
    vz = 0
    lostx = 0  #front 0 forward 1 left 0 right 1
    losty = 0
    t1 = time.time()
    while (1):
        if(time.time() -t1 > 7):
            break
        print(height.relative)
        cur_Position_Target = Intarget_local(0,0,1) 
        run_state_update()       
        time.sleep(0.02)
    print('break')
#--------------------------------------------------------
    tddl = time.time()
    while 1:
        if(flag == 1):
            break
        if(cammsg.data[0] != 0):
            x = position.pose.position.x
            y = position.pose.position.y
            z = position.pose.position.z
            t1 = time.time()
            if(time.time() - tddl >30):
                while(1):
                    if(position.pose.position.z < 0.2):
                        armServer(False)
                        flag = 1
                        break
                    cur_Position_Target = Intarget_local(x,y,0)
                    run_state_update()       
                    time.sleep(0.02)
                    print('meet ddl!')
            if(flag == 1):
                break
            if(305 < cammsg.data[2] < 335 and 230 < cammsg.data[3] < 250):
                target_h = 0
                while(1):
                    if(position.pose.position.z < 0.2):
                        armServer(False)
                        flag = 1
                        break
                    if(position.pose.position.z < target_h+0.1):
                        break
                    cur_Position_Target = Intarget_local(x,y,target_h)
                    run_state_update()       
                    time.sleep(0.02)
                    print('good!')

            else:
                if(0 < cammsg.data[2] <= 310):
                    lostx = 0
                    x -= 0.1
                    while (1):
                        if(time.time() -t1 > 0.6):
                            break
                        print(height.relative)
                        cur_Position_Target = Intarget_local(x,y,z) 
                        run_state_update()       
                        time.sleep(0.02)
                elif(cammsg.data[2]>= 330):
                    lostx = 1
                    x += 0.1
                    while (1):
                        if(time.time() -t1 > 0.6):
                            break
                        print(height.relative)
                        cur_Position_Target = Intarget_local(x,y,z) 
                        run_state_update()       
                        time.sleep(0.02)
                if(0 < cammsg.data[3] <= 230):
                    losty = 0
                    y += 0.1
                    while (1):
                        if(time.time() -t1 > 0.6):
                            break
                        print(height.relative)
                        cur_Position_Target = Intarget_local(x,y,z) 
                        run_state_update()       
                        time.sleep(0.02)
                elif(cammsg.data[3] >= 250):
                    losty =1
                    y -= 0.1
                    while (1):
                        if(time.time() -t1 > 0.6):
                            break
                        print(height.relative)
                        cur_Position_Target = Intarget_local(x,y,z) 
                        run_state_update()       
                        time.sleep(0.02)
                 
        else:
            x = position.pose.position.x
            y = position.pose.position.y
            z = position.pose.position.z
            if(lostx == 0):
                xoffset = -0.1
            else:
                xoffset = 0.1
            if(losty == 0):
                yoffset = 0.1
            else:
                yoffset = -0.1
            while (1):
                if(time.time() -t1 > 2):
                    break
                print(height.relative)
                cur_Position_Target = Intarget_local(x+xoffset,y+yoffset,1.4) 
                run_state_update()       
                time.sleep(0.02)
    flag = 0
    print('here')
    #---------------------------------------------
    
