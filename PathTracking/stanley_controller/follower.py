"""

Path tracking simulation with Stanley steering control and PID speed control.

author: Atsushi Sakai (@Atsushi_twi)

Ref:
    - [Stanley: The robot that won the DARPA grand challenge](http://isl.ecst.csuchico.edu/DOCS/darpa2005/DARPA%202005%20Stanley.pdf)
    - [Autonomous Automobile Path Tracking](https://www.ri.cmu.edu/pub_files/2009/2/Automatic_Steering_Methods_for_Autonomous_Automobile_Path_Tracking.pdf)

"""
import numpy as np
import matplotlib.pyplot as plt
import sys
import utm
import math
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from gps_common.msg import GPSFix
from scipy.spatial.transform import Rotation as R
import time as sys_time
sys.path.append("../../PathPlanning/CubicSpline/")
# udp set up
import socket
import pickle
IP = "localhost"
PORT = 5005
sock = socket.socket(socket.AF_INET,
                   socket.SOCK_DGRAM) # UDP PROTOCOL
sock.bind((IP, PORT))


class State:
    """
    Class representing the state of a vehicle.

    :param x: (float) x-coordinate
    :param y: (float) y-coordinate
    :param yaw: (float) yaw angle
    :param v: (float) speed
    """

    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
        """Instantiate the object."""
        #super(State, self).__init__()
        #self.cmd_pub = rospy.Publisher("/lawn_mower_cmd_vel",Twist,queue_size = 10)
        #self.imu_msg = Imu()
        #self.gps_msg = GPSFix()
        #self.cmd_vel= Twist()
        #self.cmd_vel.linear.x = 0
        #self.cmd_vel.angular.z = 0
        self.x = x
        self.y = y
        self.yaw =yaw
        self.v = v
        print("state initialized",self.x,self.y,self.v)
    def gps_callback(self,msg):
        self.x,self.y,__,__ = utm.from_latlon(msg.latitude,msg.longitude)
        if np.sum([self.imu_msg.orientation.w,self.imu_msg.orientation.x,self.imu_msg.orientation.y,self.imu_msg.orientation.z])!=0:

          r = R.from_quat([self.imu_msg.orientation.w,self.imu_msg.orientation.x,self.imu_msg.orientation.y,self.imu_msg.orientation.z])
          eulars = r.as_euler('zyx',degrees=False)
        else:
          eulars = np.array([0, 0, 0])
        #print(eulars[1])
        self.yaw = normalize_angle(eulars[0])
        self.v = msg.speed
        #print(msg.latitude)
    def imu_callback(self,msg):
        self.imu_msg= msg
        #print(msg.orientation.x)
    def publish(self,speed,delta):
        #delta = np.clip(delta, -max_steer, max_steer)
        self.cmd_vel.linear.x = speed
        self.cmd_vel.angular.z = np.clip(self.cmd_vel.angular.z + delta,-max_steer,max_steer)
        self.cmd_pub.publish(self.cmd_vel)
class Leader_buffer:
    def __init__(self):
        self.buffer.x = [0]
        self.buffer.y = [0]
        self.buffer.v =[0]
        self.buffer.heading = [0]
        self.curr_start_ind = 0
        self.curr_end_ind = 0
    def current_segment(self,state):
        return self.curr_start_ind, self.curr_end_ind
    def push(self,msg):
        return 0
    def pop(self,start,end):
        return 0
    def get_leader_state(self):
        data, addr = sock.recvfrom(1024) # buffer = 1024 bytes
        message = pickle.loads(data)
        print("received message:", message)

class Follower:
    def __init__(self):
        self.state = State()
        self.desired = State()
        self.leader = Leader_buffer()
    def update_state(self):
        return 0
    def update_path(self):
        return 0
    def modify_path(self):
        return 0
    def update_leader_buffer(self):
        self.leader.get_leader_state()
        return 0


if __name__ == '__main__':
    rospy.init_node('follower')
    follower = Follower()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
          follower.update_leader_buffer()
          follower.update_state()
          follower.update_path()
          follower.send_desired_state()
