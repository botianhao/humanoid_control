#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from pynput import keyboard
import threading

key_P_press_status = False
pre_key_P_press_status = False







acc_linear_x = 0.4#人形的移动加速度
acc_linear_y = 0.4#人形的移动加速度
acc_angular_z = 0.3#人形的旋转角加速度


#实际输入的是加速度的变化量而非加速度,其中该值为正值
def change_limit(cmd_vel,vel_real,acc):
    if abs(cmd_vel-vel_real) > acc:
        if cmd_vel > vel_real:
            cmd_vel = vel_real + acc
        else:
            cmd_vel = vel_real - acc
    
    return cmd_vel














class KeyboardController:
    def __init__(self):
        self.node = Node('keyboard_control')
        self.publisher = self.node.create_publisher( Twist,'/cmd_vel',1)
        self.hw_switch_publisher = self.node.create_publisher( Bool ,'/hwswitch',1)
        self.twist_msg = Twist()
        
        self.thread = 0.0

    def on_press(self, key):
        try:
            if key.char == 'z':
                # 退出程序
                rclpy.shutdown()
            else:
                # 根据按键设置线性速度和角速度
                if key.char == 'w':
                    self.twist_msg.linear.x = 0.3
                elif key.char == 's':
                    self.twist_msg.linear.x = -0.3
                else:
                    self.twist_msg.linear.x = 0.0

                if key.char == 'a':
                    self.twist_msg.linear.y = 0.1
                elif key.char == 'd':
                    self.twist_msg.linear.y = -0.1
                else:
                    self.twist_msg.linear.y = 0.0

                if key.char == 'q':
                    self.twist_msg.angular.z = 0.3
                elif key.char == 'e':
                    self.twist_msg.angular.z = -0.3
                else:
                    self.twist_msg.angular.z = 0.0

                if key.char == 'p':
                    global key_P_press_status
                    key_P_press_status = True
        except AttributeError:
            pass

    def on_release(self, key):
        # 松开按键时设置线性速度和角速度为0，并发布消息
        self.twist_msg.linear.x = 0.0
        self.twist_msg.linear.y = 0.0
        self.twist_msg.angular.z = 0.0
        try:
            if key.char == 'p':
                global key_P_press_status
                key_P_press_status = False
        except AttributeError:
            pass
    def ros_publish(self):
        rate = self.node.create_rate(150.0,self.node.get_clock())  # 设置循环的频率为150Hz
        twist_msg_last = Twist()#记录了上一次发送的twist数值
        twist_msg_last.linear.x = 0.0
        twist_msg_last.linear.y = 0.0
        twist_msg_last.angular.z = 0.0
        while rclpy.ok():

            global key_P_press_status
            global pre_key_P_press_status
            global hw_switch_bool

            twist_msg = Twist()
            twist_msg.linear.x = self.twist_msg.linear.x
            twist_msg.linear.y = self.twist_msg.linear.y
            twist_msg.angular.z = self.twist_msg.angular.z

            twist_msg.linear.x = change_limit(self.twist_msg.linear.x,twist_msg_last.linear.x,acc_linear_x/150.0)
            twist_msg.linear.y = change_limit(self.twist_msg.linear.y,twist_msg_last.linear.y,acc_linear_y/150.0)
            twist_msg.angular.z = change_limit(self.twist_msg.angular.z,twist_msg_last.angular.z,acc_angular_z/150.0)

            self.publisher.publish(twist_msg)
            twist_msg_last = twist_msg

            if key_P_press_status and not pre_key_P_press_status:
                hw_switch_bool = not hw_switch_bool
                print(f'\n Switch the output status to {hw_switch_bool}')
            pre_key_P_press_status = key_P_press_status
            hw_switch_msg = Bool()
            hw_switch_msg.data = hw_switch_bool
            self.hw_switch_publisher.publish(hw_switch_msg)
            rate.sleep()

    def start(self):
        self.thread = threading.Thread(target=self.ros_publish)
        self.thread.start()

hw_switch_bool = False




def main():
    # 初始化ROS节点
    rclpy.init()

    controller = KeyboardController()

    # 创建一个独立的线程来监听键盘按键

    controller.start()

    listener = keyboard.Listener(on_press=controller.on_press, on_release=controller.on_release)
    listener.start()
    while rclpy.ok():
        rclpy.spin_once(controller.node)

    listener.stop()
    listener.join()
    

    # 退出程序
    controller.thread.join()



if __name__ == '__main__':
    main()