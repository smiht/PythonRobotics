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
import pickle
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from std_msgs.msg import String
from gps_common.msg import GPSFix
from scipy.spatial.transform import Rotation as R
import time as sys_time
import socket
import pickle
import config as cfg
sys.path.append("../../PathPlanning/CubicSpline/")
# udp set up

#IP = "localhost"
#PORT = 5005
sock = socket.socket(socket.AF_INET,
                   socket.SOCK_DGRAM) # UDP PROTOCOL
sock.bind((cfg.RECEIVE_UDP_IP, cfg.RECEIVE_UDP_PORT))


class State:
    """
    Class representing the state of a vehicle.

    :param x: (float) x-coordinate
    :param y: (float) y-coordinate
    :param yaw: (float) yaw angle
    :param v: (float) speed
    """

    def __init__(self, x=0.0, y=0.0, heading=0.0, v=0.0):
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
        self.heading =heading
        self.v = v
        print("state initialized",self.x,self.y,' speed: ',self.v,' heading:',self.heading)
    def update_state(self,x,y,heading,speed):
        self.x = x
        self.y = y
        self.heading = heading
        self.v = speed
        print("state updated",self.x,self.y,' speed: ',self.v,' heading:',self.heading)
    def calc_distance(self, point_x, point_y):
        dx = self.x - point_x
        dy = self.y - point_y
        return math.hypot(dx, dy)

class Leader_buffer:
    def __init__(self):
        self.message_cnt = 0
        self.buffer = np.array()
        self.state = State()
        #self.buffer.y = [0]
        #self.buffer.v =[0]
        #self.buffer.heading = [0]
        self.curr_start_ind = 0
        self.curr_end_ind = -1
        self.old_nearest_point_index = None
    def get_leader_state(self):
        data, addr = sock.recvfrom(1024) # buffer = 1024 bytes
        message = pickle.loads(data)
        print('udp message received', self.message_cnt)
        self.message_cnt = self.message_cnt+1
        utm_x,utm_y,__,__= utm.from_latlon(message[0],message[1])
        self.state.update_state(x = utm_x,y = utm_y, heading = message[3], speed = message[2])
        self.buffer.append([self.state.x,self.state.y,self.state.heading,self.state.v])
        #print('end of buffer: ',self.buffer[-1].heading)
        print("length of buffer ", len(self.buffer))
        #print(self.buffer[1:5].x)
        #if len(self.buffer) > 10:
        #   print('x of entire buffer',self.buffer)
        #   del self.buffer[1:5]

        #print( message)

    def search_target_index(self, state):

        # To speed up nearest point search, doing it at only first time.
        print(self.buffer)
        return 0,0
        if self.old_nearest_point_index is None:
            # search nearest point index
            dx = [state.x - ix for ix in self.buffer[:][0]]
            dy = [state.y - iy for iy in self.buffer[:][1]]
            d = np.hypot(dx, dy)
            ind = np.argmin(d)
            self.old_nearest_point_index = ind
        else:
            ind = self.old_nearest_point_index
            distance_this_index = state.calc_distance(self.buffer[ind][0],
                                                      self.buffer[ind][1])
            while True:
                distance_next_index = state.calc_distance(self.buffer[ind + 1][0],
                                                          self.buffer[ind + 1][1])
                if distance_this_index < distance_next_index:
                    break
                ind = ind + 1 if (ind + 1) < len(self.buffer[:,0]) else ind
                distance_this_index = distance_next_index
            self.old_nearest_point_index = ind
        return ind,distance_this_index

class Follower:
    def __init__(self):
        self.state = State()
        self.desired = State()
        self.leader = Leader_buffer()
        self.path = []
    def update_state(self,msg):
        utm_x,utm_y,__,__ = utm.from_latlon(msg.latitude,msg.longitude)
        self.state.update_state(x = utm_x,y = utm_y ,heading = msg.track, speed = msg.speed)

    def get_send_desired_state(self):
        closest_index, distance = self.leader.search_target_index(self.state)
        #self.path = self.leader.buffer[closest_index:,]
        # for ploting purpose
        #self.desired.x,self.desired.y,self.desired.heading,self.desired.v = self.leader.buffer[closest_index]
        print(self.desired)
        # send the udp to houshiyar
        message = pickle.dumps([self.desired.x,self.desired.y,self.desired.heading,self.desired.v,distance])
        sock.sendto(message, (cfg.SEND_UDP_IP, cfg.SEND_UDP_PORT))
    def update_path(self,msg):
        print(msg)
    def modify_path(self):
        return 0

    def update_leader_buffer(self):
        self.leader.get_leader_state()


if __name__ == '__main__':
    rospy.init_node('follower')
    follower = Follower()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
          follower.update_leader_buffer()
          rospy.Subscriber('/gps', GPSFix,follower.update_state)
          rospy.Subscriber('Follower/msg',String, follower.update_path)
          follower.get_send_desired_state()
          #follower.update_path()
