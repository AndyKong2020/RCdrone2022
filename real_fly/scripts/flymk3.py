#!/usr/bin/env python
import rospy
import os
from geometry_msgs.msg import Twist
import sys, select, termios, tty
import rospy
from mavros_msgs.msg import PositionTarget, State ,HomePosition ,Altitude
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import Imu, NavSatFix
from std_msgs.msg import Float32, String, Float32MultiArray, Int32
import time
import math


message = """
$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
%%%%%%%%%%%%%%%%%%%%%%%
command_cotrol
%%%%%%%%%%%%%%%%%%%%%%%
1 : TAKEOFF
---------------------------

CTRL-C to quit

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
    rospy.Subscriber("/mavros/state", State, mavros_state_callback)
    print("Initialized")

def callback(msg):
    global cammsg
    cammsg = msg

def mavros_state_callback(msg):
    global mavros_state
    mavros_state = msg
    
def Intarget_local(x,y,z):   #,vx,vy,vz,afx,afy,afz

    set_target_local = PositionTarget()
    set_target_local.type_mask = 0b100111000000
    set_target_local.coordinate_frame = 8
    set_target_local.position.x = x
    set_target_local.position.y = y
    set_target_local.position.z = z
    set_target_local.yaw = 0
    return set_target_local


def VC():   #,vx,vy,vz,afx,afy,afz
    set_target_local = PositionTarget()
    set_target_local.type_mask = 0b110111000111             #设置忽略坐标信息
    set_target_local.coordinate_frame = 8               #设置坐标系
    set_target_local.velocity.x = vx                #设置速度
    set_target_local.velocity.y = vy
    set_target_local.velocity.z = vz
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

def takeoff():
    while (height.relative < 2.2):
        vx = 0
        vy = 0
        vz = 2
        print(height.relative)
        cur_Position_Target = VC()      #把要发送的信息设置为速度信息
        run_state_update()          #发送信息
        time.sleep(0.2)

def find(target):
    while 1:
        if(cammsg.data[0] == target):
            print(cammsg.data[0])
            while 1:
                vx = 0
                vy = 0
                vz = 0
                V = height.relative*0.1
                print(cammsg.data[2],cammsg.data[3],height.relative)
                if height.relative > 1:
                    if(230 < cammsg.data[2] < 410 and 180 < cammsg.data[3] < 300):
                        vz = -V*10
                    else:
                        vz = 0
                        if(0 < cammsg.data[2] <= 213):
                            vx = -1.5*V
                        elif(cammsg.data[2]>= 426):
                            vx = 1.5*V
                        if(0 < cammsg.data[3] <= 210):
                            vy = V
                        elif(cammsg.data[3] >= 270):
                            vy = -V
                elif(0.5 < height.relative <= 1):
                    if(305 < cammsg.data[2] < 335 and 230 < cammsg.data[3] < 250):
                        vz = -0.1
                    else:
                        vz = 0
                        if(0 < cammsg.data[2] <= 310):
                            vx = -2*V*(((310-cammsg.data[2])*(310-cammsg.data[2])*(310-cammsg.data[2])/5000000)+0.9)
                        elif(cammsg.data[2]>= 330):
                            vx = 2*V*(((cammsg.data[2]-330)*(cammsg.data[2]-330)*(cammsg.data[2]-330)/5000000)+0.9)
                        if(0 < cammsg.data[3] <= 230):
                            vy = 1.5*V*(((230-cammsg.data[3])*(230-cammsg.data[3])*(230-cammsg.data[3])/5000000)+0.9)
                        elif(cammsg.data[3] >= 250):
                            vy = -1.5*V*(((cammsg.data[3]-250)*(cammsg.data[3]-250)*(cammsg.data[3]-250)/5000000)+0.9)
                else:
                    vz = -0.5
                    print("landing....")
                print(height.relative,vx,vy,vz)
                cur_Position_Target = VC()
                run_state_update()
                time.sleep(0.2)
                if (mavros_state.armed == False):
                    break
            break


def turn(a):
    while(cammsg.data[0] != target):
        sinmt = math.sin(math.radians(a))
        vx = 0.3 * math.sin(math.radians(a))
        vy = 0.3 * math.cos(math.radians(a))
        vz = 0
        cur_Position_Target = VC()   
        run_state_update()         
        time.sleep(0.2)

def initialize():
    armServer(True)
    setModeServer(custom_mode='OFFBOARD')


cur_Position_Target = VC()




if __name__=="__main__":
    __init__()
    print(message)
    initialize()
    vx = 0
    vy = 0
    vz = 0
#===========================PART . 1 BEGIN==============================================================================
    #-------------------------------------------------------

#takeoff
    while (height.relative < 2.2):
        vx = 0
        vy = 0
        vz = 2
        print(height.relative)
        cur_Position_Target = VC()  
        run_state_update()       
        time.sleep(0.2)
        #-----------------------------------------------------
#find
        target = 2
    while 1:
        if(cammsg.data[0] == target):
            print(cammsg.data[0])
            while 1:
                vx = 0
                vy = 0
                vz = 0
                V = height.relative*0.1
                print(cammsg.data[2],cammsg.data[3],height.relative)
                if height.relative > 1:
                    if(230 < cammsg.data[2] < 410 and 180 < cammsg.data[3] < 300):
                        vz = -V*10
                    else:
                        vz = 0
                        if(0 < cammsg.data[2] <= 213):
                            vx = -1.5*V
                        elif(cammsg.data[2]>= 426):
                            vx = 1.5*V
                        if(0 < cammsg.data[3] <= 210):
                            vy = V
                        elif(cammsg.data[3] >= 270):
                            vy = -V
                elif(0.5 < height.relative <= 1):
                    if(305 < cammsg.data[2] < 335 and 230 < cammsg.data[3] < 250):
                        vz = -0.1
                    else:
                        vz = 0
                        if(0 < cammsg.data[2] <= 310):
                            vx = -2*V*(((310-cammsg.data[2])*(310-cammsg.data[2])*(310-cammsg.data[2])/5000000)+0.9)
                        elif(cammsg.data[2]>= 330):
                            vx = 2*V*(((cammsg.data[2]-330)*(cammsg.data[2]-330)*(cammsg.data[2]-330)/5000000)+0.9)
                        if(0 < cammsg.data[3] <= 230):
                            vy = 1.5*V*(((230-cammsg.data[3])*(230-cammsg.data[3])*(230-cammsg.data[3])/5000000)+0.9)
                        elif(cammsg.data[3] >= 250):
                            vy = -1.5*V*(((cammsg.data[3]-250)*(cammsg.data[3]-250)*(cammsg.data[3]-250)/5000000)+0.9)
                else:
                    vz = -0.5
                    print("landing....")
                print(height.relative,vx,vy,vz)
                cur_Position_Target = VC()
                run_state_update()
                time.sleep(0.2)
                if (mavros_state.armed == False):
                    break
            break
            #---------------------------------------------
#===========================PART . 1 END==============================================================================

#===========================PART . 2 BEGIN============================================================================ 
    initialize()
    print("1111111111111111")
 #takeoff
    while (height.relative < 1.2):
        vx = 0
        vy = 0
        vz = 2
        print(height.relative)
        cur_Position_Target = VC()     
        run_state_update()        
        time.sleep(0.2)
        #-----------------------------------------------------
#turn
    target = 1
    a = 0
    #=========================MODIFY  TARGET & A=======================================
    while(cammsg.data[0] != target):
        sinmt = math.sin(math.radians(a))
        vx = 0.3 * math.sin(math.radians(a))
        vy = 0.3 * math.cos(math.radians(a))
        vz = 0
        cur_Position_Target = VC()    
        run_state_update()        
        time.sleep(0.2)
        #-----------------------------------------------------
#find
    while 1:
        if(cammsg.data[0] == target):
            print(cammsg.data[0])
            while 1:
                vx = 0
                vy = 0
                vz = 0
                V = height.relative*0.1
                print(cammsg.data[2],cammsg.data[3],height.relative)
                if height.relative > 1:
                    if(230 < cammsg.data[2] < 410 and 180 < cammsg.data[3] < 300):
                        vz = -V*10
                    else:
                        vz = 0
                        if(0 < cammsg.data[2] <= 213):
                            vx = -1.5*V
                        elif(cammsg.data[2]>= 426):
                            vx = 1.5*V
                        if(0 < cammsg.data[3] <= 210):
                            vy = V
                        elif(cammsg.data[3] >= 270):
                            vy = -V
                elif(0.5 < height.relative <= 1):
                    if(305 < cammsg.data[2] < 335 and 230 < cammsg.data[3] < 250):
                        vz = -0.1
                    else:
                        vz = 0
                        if(0 < cammsg.data[2] <= 310):
                            vx = -2*V*(((310-cammsg.data[2])*(310-cammsg.data[2])*(310-cammsg.data[2])/5000000)+0.9)
                        elif(cammsg.data[2]>= 330):
                            vx = 2*V*(((cammsg.data[2]-330)*(cammsg.data[2]-330)*(cammsg.data[2]-330)/5000000)+0.9)
                        if(0 < cammsg.data[3] <= 230):
                            vy = 1.5*V*(((230-cammsg.data[3])*(230-cammsg.data[3])*(230-cammsg.data[3])/5000000)+0.9)
                        elif(cammsg.data[3] >= 250):
                            vy = -1.5*V*(((cammsg.data[3]-250)*(cammsg.data[3]-250)*(cammsg.data[3]-250)/5000000)+0.9)
                else:
                    vz = -0.5
                    print("landing....")
                print(height.relative,vx,vy,vz)
                cur_Position_Target = VC()
                run_state_update()
                time.sleep(0.2)
                if (mavros_state.armed == False):
                    break
            break
        #-------------------------------
#===========================PART . 2 END================================================================================


#===========================PART . 3 BEGIN==============================================================================
    initialize()
    print("1111111111111111")
 #takeoff
    while (height.relative < 2.2):
        vx = 0
        vy = 0
        vz = 2
        print(height.relative)
        cur_Position_Target = VC()     
        run_state_update()        
        time.sleep(0.2)
        #-----------------------------------------------------
#turn
    target = 2
    a = 120
    #=========================MODIFY  TARGET & A=======================================
    while(cammsg.data[0] != target):
        sinmt = math.sin(math.radians(a))
        vx = 0.3 * math.sin(math.radians(a))
        vy = 0.3 * math.cos(math.radians(a))
        vz = 0
        cur_Position_Target = VC()    
        run_state_update()        
        time.sleep(0.2)
        #-----------------------------------------------------
#find
    while 1:
        if(cammsg.data[0] == target):
            print(cammsg.data[0])
            while 1:
                vx = 0
                vy = 0
                vz = 0
                V = height.relative*0.1
                print(cammsg.data[2],cammsg.data[3],height.relative)
                if height.relative > 1:
                    if(230 < cammsg.data[2] < 410 and 180 < cammsg.data[3] < 300):
                        vz = -V*10
                    else:
                        vz = 0
                        if(0 < cammsg.data[2] <= 213):
                            vx = -1.5*V
                        elif(cammsg.data[2]>= 426):
                            vx = 1.5*V
                        if(0 < cammsg.data[3] <= 210):
                            vy = V
                        elif(cammsg.data[3] >= 270):
                            vy = -V
                elif(0.5 < height.relative <= 1):
                    if(305 < cammsg.data[2] < 335 and 230 < cammsg.data[3] < 250):
                        vz = -0.1
                    else:
                        vz = 0
                        if(0 < cammsg.data[2] <= 310):
                            vx = -2*V*(((310-cammsg.data[2])*(310-cammsg.data[2])*(310-cammsg.data[2])/5000000)+0.9)
                        elif(cammsg.data[2]>= 330):
                            vx = 2*V*(((cammsg.data[2]-330)*(cammsg.data[2]-330)*(cammsg.data[2]-330)/5000000)+0.9)
                        if(0 < cammsg.data[3] <= 230):
                            vy = 1.5*V*(((230-cammsg.data[3])*(230-cammsg.data[3])*(230-cammsg.data[3])/5000000)+0.9)
                        elif(cammsg.data[3] >= 250):
                            vy = -1.5*V*(((cammsg.data[3]-250)*(cammsg.data[3]-250)*(cammsg.data[3]-250)/5000000)+0.9)
                else:
                    vz = -0.5
                    print("landing....")
                print(height.relative,vx,vy,vz)
                cur_Position_Target = VC()
                run_state_update()
                time.sleep(0.2)
                if (mavros_state.armed == False):
                    break
            break
        #-------------------------------
#===========================PART . 2 BEGIN==========================================================================





