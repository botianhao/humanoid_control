#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
import threading
from std_msgs.msg import Bool
from std_msgs.msg import String
from rclpy.time import Time, Duration
vx = 0.0
vy = 0.0
w = 0.0
key_Y_pressed = False
pre_key_Y_press_status = False
key_X_pressed = False
pre_key_X_press_status = False

def callback(data):
    global vx, vy, w, key_Y_pressed, key_X_pressed
    vx = data.axes[1] * 0.1
    vy = data.axes[0] * 0.02
    w = data.axes[2] * 0.3
    if data.buttons[3] == 1:
        key_Y_pressed = True
    else:
        key_Y_pressed = False
    if data.buttons[0] == 1:
        key_X_pressed = True
    else:
        key_X_pressed = False

# ros init
rclpy.init()
node = Node('joy11')        

cmd_pub = node.create_publisher(Twist,'/cmd_vel',1)
node.create_subscription(Joy,'joy',callback,10)

publisher = node.create_publisher(Twist, '/cmd_vel',1)

rate = node.create_rate(150)
hw_switch_bool = False
hw_switch_publisher = node.create_publisher(Bool, '/hwswitch', 1)

gait_str = "walk"
gait_str_publisher = node.create_publisher(String, '/desired_gait_str', 1)


acc_linear_x = 0.5#人形的移动加速度
acc_linear_y = 0.5#人形的移动加速度
acc_angular_z = 0.4#人形的旋转角加速度


#实际输入的是加速度的变化量而非加速度,其中该值为正值
def change_limit(cmd_vel,vel_real,acc):
    if abs(cmd_vel-vel_real) > acc:
        if cmd_vel > vel_real:
            cmd_vel = vel_real + acc
        else:
            cmd_vel = vel_real - acc
    
    return cmd_vel
         



def ros_publish():
    global key_Y_pressed
    global pre_key_Y_press_status
    global hw_switch_bool
    global key_X_pressed
    global gait_str

    twist_msg_last = Twist()#记录了上一次发送的twist数值
    twist_msg_last.linear.x = 0.0
    twist_msg_last.linear.y = 0.0
    twist_msg_last.angular.z = 0.0
    while rclpy.ok():
        if( hw_switch_bool == True):

            twist_msg = Twist()
            twist_msg.linear.x = vx
            twist_msg.linear.y = vy
            twist_msg.angular.z = w

            twist_msg.linear.x = change_limit(vx,twist_msg_last.linear.x,acc_linear_x/150.0)
            twist_msg.linear.y = change_limit(vy,twist_msg_last.linear.y,acc_linear_y/150.0)
            twist_msg.angular.z = change_limit(w,twist_msg_last.angular.z,acc_angular_z/150.0)

            publisher.publish(twist_msg)
            twist_msg_last = twist_msg#存储上一次发送的速度
        else:
            twist_msg = Twist()
            twist_msg.linear.x = 0.0
            twist_msg.linear.y = 0.0
            twist_msg.angular.z = 0.0

            publisher.publish(twist_msg)
            twist_msg_last = twist_msg#存储上一次发送的速度            




        if key_Y_pressed and not pre_key_Y_press_status:
            hw_switch_bool = not hw_switch_bool
            print(f'\n Switch the output status to {hw_switch_bool}')
        pre_key_Y_press_status = key_Y_pressed

        hw_switch_msg = Bool()
        hw_switch_msg.data = hw_switch_bool
        hw_switch_publisher.publish(hw_switch_msg)

        if key_X_pressed and not pre_key_X_press_status:
            if gait_str == "trot":
                gait_str = "walk"
            elif gait_str == "walk":
                gait_str = "trot"
            print(f'\n Switch the desire gait to {gait_str}')
        pre_key_X_press_status = key_X_pressed

        gait_str_msg = String()
        gait_str_msg.data = gait_str
        gait_str_publisher.publish(gait_str_msg)

        rate.sleep()

thread = threading.Thread(target=ros_publish)
thread.start()

rclpy.spin(node)
rclpy.shutdown()
