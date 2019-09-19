#!/usr/bin/env python
import roslib; roslib.load_manifest('ur_driver')
import time, sys, threading, math
import copy
import datetime
import socket, select
import struct
import traceback, code
import SocketServer
import time
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from ur_driver.deserializeRT import RobotStateRT
from ur_msgs.msg import *
from opencv_bridge.msg import *
import moveit_msgs.msg

joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
               'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
joint_offsets = {}


class UrRobot:

    def __init__(self):
        rospy.init_node('ur_robot', disable_signals=True)
        self.robot_hostname = '192.168.1.99'
        self.rt_port = 30003
        self.rt_socket = socket.create_connection((self.robot_hostname, self.rt_port))
        self.pub_joint_statesRT = rospy.Publisher('joint_statesRT', JointState, queue_size=1)
        self.pub_robot_stateRT = rospy.Publisher('robot_stateRT', RobotStateRTMsg, queue_size=1)
        self.pub_RobotMsg = rospy.Publisher('ur', RobotMsg, queue_size=1)
        self.rate = rospy.Rate(50)
        self.buf = ""
        self.tcp_pose = []
        rospy.Subscriber("command",  Command, self.callback_robot)

    def callback_robot(self,data_cmd):
        if data_cmd.type == 0:
            #print("empty command")
            return 
        elif data_cmd.type == 1:
            self.move_line_p(data_cmd)
            print("move line")
        elif data_cmd.type == 2:
            self.move_joint_p(data_cmd)
            print("move joint, input is a pose")
        elif data_cmd.type == 3:
            self.io_control(data_cmd)
            print("io control")
        elif data_cmd.type == 4:
            self.stop()
            print("stop")
        elif data_cmd.type == 5:
            self.move_tool(data_cmd)
            print("move tool")
        elif data_cmd.type == 6:
            self.move_joint_multi(data_cmd)
            print("move trajectory in joint space")
        elif data_cmd.type == 7:
            self.move_line_multi(data_cmd)
            print("move trajectory in joint space")
        elif data_cmd.type == 9:
            self.move_joint_j(data_cmd)
            print("move joint, input is joint angle")

    def recieve(self):
        while not rospy.is_shutdown():
            more = self.rt_socket.recv(4096)
            if more:
                self.buf = self.buf + more
            # Attempts to extract a packet
            packet_length = struct.unpack_from("!i", self.buf)[0]
            #print("PacketLength: ", packet_length, "; BufferSize: ", len(buf))
            if len(self.buf) >= packet_length:
                packet, self.buf = self.buf[:packet_length], self.buf[packet_length:]
                self.__on_packet(packet)
                #self.rate.sleep()
            else:
                print("There is no more...")
                
    def __on_packet(self,buf):
        stateRT = RobotStateRT.unpack(buf)
        robot_msg = RobotMsg()
        robot_msg.header.stamp = rospy.Time.now()

        msg = RobotStateRTMsg()
        msg.time = stateRT.time
        msg.q_target = stateRT.q_target
        msg.qd_target = stateRT.qd_target
        msg.qdd_target = stateRT.qdd_target
        msg.i_target = stateRT.i_target
        msg.m_target = stateRT.m_target
        msg.q_actual = stateRT.q_actual
        msg.qd_actual = stateRT.qd_actual
        msg.i_actual = stateRT.i_actual
        msg.tool_acc_values = stateRT.tool_acc_values
        msg.tcp_force = stateRT.tcp_force
        msg.tool_vector = stateRT.tool_vector_actual
        msg.tcp_speed = stateRT.tcp_speed_actual
        msg.digital_input_bits = stateRT.digital_input_bits
        msg.motor_temperatures = stateRT.motor_temperatures
        msg.controller_timer = stateRT.controller_timer
        msg.test_value = stateRT.test_value
        msg.robot_mode = stateRT.robot_mode
        msg.joint_modes = stateRT.joint_modes

        robot_msg.data = msg;
        self.pub_RobotMsg.publish(robot_msg)
        self.pub_robot_stateRT.publish(msg)       
        self.tcp_pose = msg.tool_vector

        print('joint:')
        print(msg.q_actual)
        print('ee pose:')
        print(self.tcp_pose)
        print('\n')
        
        msg = JointState()
        msg.header.stamp = rospy.get_rostime()
        msg.header.frame_id = "From real-time state data"
        msg.name = joint_names
        msg.position = [0.0] * 6
        for i, q in enumerate(stateRT.q_actual):
            msg.position[i] = q + joint_offsets.get(joint_names[i], 0.0)
        msg.velocity = stateRT.qd_actual
        msg.effort = [0]*6
        self.pub_joint_statesRT.publish(msg)
        #print(msg.position)

    def move_line_p(self,cmd):
        if not len(cmd.pose) == 6:
            print("Command length error!")
            return 
        if not (cmd.speed > 0 and cmd.acce > 0):
            print("Command speed or acceleration error!")
            return 
        cmd_str = "movel(p[" + str(cmd.pose[0]) + ","\
                    + str(cmd.pose[1]) + ","\
                    + str(cmd.pose[2]) + ","\
                    + str(cmd.pose[3]) + ","\
                    + str(cmd.pose[4]) + ","\
                    + str(cmd.pose[5]) + "],a="\
                    + str(cmd.acce) + ",v="\
                    + str(cmd.speed)+ ",t=" + str(cmd.time) + ")\n"
        print(cmd_str)
        self.rt_socket.send(cmd_str)

    def move_joint_p(self,cmd):
        if not len(cmd.pose) == 6:
            print("Command length error!")
            return 
        if not (cmd.speed > 0 and cmd.acce > 0):
            print("Command speed or acceleration error!")
            return 
        cmd_str = "movej(p[" + str(cmd.pose[0]) + ","\
                    + str(cmd.pose[1]) + ","\
                    + str(cmd.pose[2]) + ","\
                    + str(cmd.pose[3]) + ","\
                    + str(cmd.pose[4]) + ","\
                    + str(cmd.pose[5]) + "],a="\
                    + str(cmd.acce) + ",v="\
                    + str(cmd.speed)+ ",t=" + str(cmd.time) + ")\n"
        print(cmd_str)
        self.rt_socket.send(cmd_str)

    def move_joint_j(self,cmd):
        if not len(cmd.joint) == 6:
            print("Command length error!")
            return 
        if not (cmd.speed > 0 and cmd.acce > 0):
            print("Command speed or acceleration error!")
            return 
        cmd_str = "movej([" + str(cmd.joint[0]) + ","\
                    + str(cmd.joint[1]) + ","\
                    + str(cmd.joint[2]) + ","\
                    + str(cmd.joint[3]) + ","\
                    + str(cmd.joint[4]) + ","\
                    + str(cmd.joint[5]) + "],a="\
                    + str(cmd.acce) + ",v="\
                    + str(cmd.speed)+ ",t=" + str(cmd.time) + ")\n"
        print(cmd_str)
        self.rt_socket.send(cmd_str)

    def io_control(self,cmd):
        if cmd.io == 0:
            print("io error!")
            return
        if cmd.io > 0:
            cmd_str = "set_digital_out(" + str(cmd.io) + ",True)\n"
        elif cmd.io < 0:
            cmd_str = "set_digital_out(" + str(cmd.io*(-1)) + ",False)\n"
        print(cmd_str)
        self.rt_socket.send(cmd_str)

    def stop(self):
        cmd_str = "stopl(1.5)\n"
        print(cmd_str)
        self.rt_socket.send(cmd_str)

    def move_tool(self,cmd):
        if not len(cmd.delta_pose) == 6:
            print("Command length error!")
            return 
        if not (cmd.speed > 0 and cmd.acce > 0):
            print("Command speed or acceleration error!")
            return 
        delta = "p["\
                + str(cmd.delta_pose[0]) + ","\
                + str(cmd.delta_pose[1]) + ","\
                + str(cmd.delta_pose[2]) + ","\
                + str(cmd.delta_pose[3]) + ","\
                + str(cmd.delta_pose[4]) + ","\
                + str(cmd.delta_pose[5]) + "]"

        cmd_str = "movej(pose_trans(p[" + str(self.tcp_pose[0]) + ","\
                    + str(self.tcp_pose[1]) + ","\
                    + str(self.tcp_pose[2]) + ","\
                    + str(self.tcp_pose[3]) + ","\
                    + str(self.tcp_pose[4]) + ","\
                    + str(self.tcp_pose[5]) + "]," + delta\
                    + "),a=" + str(cmd.acce) + ",v="\
                    + str(cmd.speed)+ ",t=" + str(cmd.time) + ")\n"
        print(cmd_str)
        self.rt_socket.send(cmd_str)

    #INPUT:  JOINT
    def move_joint_multi(self,cmd):  
        cmd_str = str('def foo():')
        for num in range(0,cmd.road_point):
            cmd_str = cmd_str + "movej([" + str(cmd.pose_array.data[9*num + 0]) + ","\
                                    + str(cmd.pose_array.data[9*num + 1]) + ","\
                                    + str(cmd.pose_array.data[9*num + 2]) + ","\
                                    + str(cmd.pose_array.data[9*num + 3]) + ","\
                                    + str(cmd.pose_array.data[9*num + 4]) + ","\
                                    + str(cmd.pose_array.data[9*num + 5]) + "],a="\
                                    + str(cmd.pose_array.data[9*num + 6]) + ",v="\
                                    + str(cmd.pose_array.data[9*num + 7]) + ",t="\
                                    + str(cmd.pose_array.data[9*num + 8]) +",r=0)\n"
        cmd_str = cmd_str + "end\n"
        print(cmd_str)
        self.rt_socket.send(cmd_str)

    #INPUT:  POSE
    def move_line_multi(self,cmd):
        cmd_str = str('def foo():')
        for num in range(0,cmd.road_point):
            cmd_str = cmd_str + "movel(p[" + str(cmd.pose_array.data[9*num + 0]) + ","\
                                    + str(cmd.pose_array.data[9*num + 1]) + ","\
                                    + str(cmd.pose_array.data[9*num + 2]) + ","\
                                    + str(cmd.pose_array.data[9*num + 3]) + ","\
                                    + str(cmd.pose_array.data[9*num + 4]) + ","\
                                    + str(cmd.pose_array.data[9*num + 5]) + "],a="\
                                    + str(cmd.pose_array.data[9*num + 6]) + ",v="\
                                    + str(cmd.pose_array.data[9*num + 7]) + ",t="\
                                    + str(cmd.pose_array.data[9*num + 8]) +",r=0)\n"
        cmd_str = cmd_str + "end\n"
        print(cmd_str)
        self.rt_socket.send(cmd_str)

#movej(pose_trans(p[-0.66814294568,0.026965790173,0.408793334779,-2.76953631893,-1.43000788128,-0.00561014481964],p[0.0,0.0,0.10000000149,0.0,0.0,0.0]),a=1.0,v=2.0)
    def pick_place(self,cmd):
        cmd_str = str('def foo():')
        delta = "p[0,0,0.2,0,0,0]"
        cmd_str = cmd_str + "movej(p[-0.6673332179591891, 0.024617266711594504, 0.33156365575448715, -2.7695717556302233, -1.4299823090047217, -0.005395799166463273],a=0.5,v=0.5,t=0,r=0)\n"\
                         + "movej(pose_trans(p[" + str(self.tcp_pose[0]) + ","\
                    + str(self.tcp_pose[1]) + ","\
                    + str(self.tcp_pose[2]) + ","\
                    + str(self.tcp_pose[3]) + ","\
                    + str(self.tcp_pose[4]) + ","\
                    + str(self.tcp_pose[5]) + "]," + delta\
                    + "),a=" + str(cmd.acce) + ",v="\
                    + str(cmd.speed)+ ")\n"
        cmd_str = cmd_str + "end\n"
        print(cmd_str)
        self.rt_socket.send(cmd_str)
        
def main():
    robot = UrRobot()
    robot.recieve()
    rospy.spin()
    ##while not rospy.is_shutdown():

        # raw_input_commond = raw_input("commond: ")
        # if raw_input_commond == 'a':
        #      print("go")
        #      rt_socket.send("movel(p[-0.619,0.4,0.6,3.01,0.25,-0.18], a=1.2, v=0.1)\n")
    #rt_socket.close()

if __name__ == '__main__': 
    main()

