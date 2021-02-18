#!/usr/bin/env python
# -*- coding:utf-8 -*-
import rospy

import atexit
import os
import signal
from threading import Lock
from Tkinter import Frame, Label, Tk

from sensor_msgs.msg import Imu,JointState

from ackermann_msgs.msg import AckermannDriveStamped

import sys, select, termios, tty
import time


msg = """
Control Your robot!
Reading from the keyboard  and Publishing to Twist!
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

For Holonomic mode (strafing), hold down the shift key:
---------------------------
   U    I    O
   J    K    L
   M    <    >

space key, k : force stop
w/x: shift the middle pos of throttle by +/- 5 pwm
a/d: shift the middle pos of steering by +/- 2 pwm
CTRL-C to quit
"""
#throttle 油门
#steering 方向盘

#字典类型，用于控制方向

moveBindings = {
        'u':(1,1),
        'i':(1,0),
        'o':(1,-1),
        'j':(0,1),
        'k':(0,0),
        'l':(0,-1),
        'm':(-1,1),
        ',':(-1,0),
        '.':(-1,-1),
        'U':(1,1),
        'I':(1,0),
        'O':(1,-1),
        'J':(0,1),
        'K':(0,0),
        'L':(0,-1),
        'M':(-1,1),
        '<':(-1,0),
        '>':(-1,-1),
            }


def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

#控制台输出速度和转向参数
def vels(speed,turn):
    return "currently:\tspeed[ %s ]\tturn[ %s ]" % (speed,turn)

def Add_vels(speed_add_once,turn_add_once):
    return "AddOnce:\tSpeed[ %s ]\tTurn[ %s ]" % (speed_add_once,turn_add_once)

#这里为什么使用if
if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)

    #命名节点名称   
    rospy.init_node('stdr_teleop_control')
    #发布话题名称
    pub = rospy.Publisher('/racecar/ackermann_cmd', AckermannDriveStamped, queue_size=10)

    #参数设置
    speed_start_value = 1.5 
    turn_start_value = 0.5
    speed_mid = 0
    turn_mid = 0
    speed_bias = 0
    turn_bias = 0
    speed_add_once = 0.05
    turn_add_once = 0.02
    control_speed = speed_mid
    control_turn = turn_mid
    speed_dir = 0
    last_speed_dir = 0
    last_turn_dir = 0
    last_control_speed = control_speed
    last_control_turn = control_turn

    run = 0

    try:
        while(1):
            key = getKey();
            msg = AckermannDriveStamped();
            if key in moveBindings.keys():
                #   moveBindings是自定义的字典类型，用于设置转向控制参数
                run = 1 #开始
                #print "key =",key

                #speed_dir控制前进后退
                #speed_turn控制转弯方向
                speed_dir = moveBindings[key][0]
                speed_turn = moveBindings[key][1]

                #Reverse 
                # 场景1:按下J,K,L键,既不前进也不后退
                # 场景2:当前状态的前进方向 与前一帧刚好相反
               # if(speed_dir==0 or speed_dir + last_speed_dir == 0):
                if(speed_dir==0):
                    control_speed = 0            
                  #  control_turn = turn_mid
                    print "============= Reverse Speed_DIR ============"
                else:
                    if(speed_dir > 0):
                        control_speed = speed_dir * (speed_start_value + speed_bias) + speed_mid 
                    #
                    elif(speed_dir < 0):
                        control_speed = speed_dir * (speed_start_value + speed_bias) + speed_mid - 0
                    else:
                        control_speed = 0

            #    if(speed_turn==0 or speed_turn + last_speed_dir == 0):
                if(speed_turn==0):
                    control_turn = 0
                    print "============= Reverse Speed_TURN =============="
                else:
                    control_turn = speed_turn * (turn_start_value + turn_bias)+ turn_mid 
                    #print " "
                #print "speed_dir = ",speed_dir
                #print "control_speed = ",control_speed
                #print "control_turn = ",control_turn     
                last_speed_dir = speed_dir
                last_control_speed = control_speed
                last_control_turn = control_turn
            elif key == ' ' :
     #       elif key == ' ' or key == 'k' :
                speed_dir = -speed_dir #for ESC Forward/Reverse with brake mode
                control_speed = last_control_speed * speed_dir
                control_turn = turn_mid
                run = 0
            elif key == 'w' :
                speed_bias += speed_add_once              
                if(speed_bias >= 1.5):  
                   # speed_bias = 1.5
                    speed_bias = speed_bias
                else:
                    last_control_speed = last_control_speed+speed_add_once
                run = 0
            elif key == 's' :
                speed_bias -= speed_add_once
                last_control_speed = last_control_speed-speed_add_once
                if(speed_bias <= -0.5):  
                    speed_bias = -0.5
                run = 0
            elif key == 'a' :
                turn_bias += turn_add_once
                last_control_turn = last_control_turn+turn_add_once 
                if(turn_bias >= 0.5):  
                    turn_bias = turn_bias
                    turn_bias = 0.5
                run = 0
            elif key == 'd' :
                turn_bias -= turn_add_once
                last_control_turn = last_control_turn-turn_add_once 
                if(turn_bias <= -0.5):  
                    turn_bias = -0.5
                run = 0
            else:
                run = 0   
            print vels(control_speed,control_turn)  
            print Add_vels(speed_bias,turn_bias)  
            print "-----------------------------------------" 
            #print "speed_dir=",speed_dir,"last_speed_dir",last_speed_dir
            if(run == 1):                      
                msg.header.stamp = rospy.Time.now();
                msg.header.frame_id = "base_link";
                msg.drive.speed = control_speed;
                msg.drive.acceleration = 1;
                msg.drive.jerk = 1;
                msg.drive.steering_angle = control_turn;
                msg.drive.steering_angle_velocity = 1;
                pub.publish(msg);

            else:              
                msg.header.stamp = rospy.Time.now();
                msg.header.frame_id = "base_link";
                msg.drive.speed = 0;
                msg.drive.acceleration = 1;
                msg.drive.jerk = 1;
                msg.drive.steering_angle = 0;
                msg.drive.steering_angle_velocity = 1;
                pub.publish(msg);

            if (key == '\x03'):   #for ctrl + c exit
                break

    except:
        print "error"

    finally:
        msg.header.stamp = rospy.Time.now();
        msg.header.frame_id = "base_link";
        msg.drive.speed = 0;
        msg.drive.acceleration = 0;
        msg.drive.jerk = 0;
        msg.drive.steering_angle = 0;
        msg.drive.steering_angle_velocity = 0;
        pub.publish(msg);

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)