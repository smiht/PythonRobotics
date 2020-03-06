"""

Path tracking simulation with pure pursuit steering and PID speed control.

author: Atsushi Sakai (@Atsushi_twi)
        Guillaume Jacquenot (@Gjacquenot)

"""
import numpy as np
import math
import matplotlib.pyplot as plt
import utm
import sys
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from gps_common.msg import GPSFix
from scipy.spatial.transform import Rotation as R
# Parameters
k = 0.1  # look forward gain
Lfc = 1.0  # [m] look-ahead distance
Kp = 1.0  # speed proportional gain
dt = 0.1  # [s] time tick
WB = 2.9  # [m] wheel base of vehicle

show_animation = True
two_line_angle = False
sys.path.append("../../PathPlanning/CubicSpline/")

try:
    import cubic_spline_planner
except:
    raise

class State:
    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
        self.cmd_pub = rospy.Publisher("/lawn_mower_cmd_vel",Twist,queue_size = 10)
        self.imu_msg = Imu()
        self.gps_msg = GPSFix()
        self.cmd_vel= Twist()
        self.cmd_vel.linear.x = 0
        self.cmd_vel.angular.z = 0
        self.x = x
        self.y = y
        self.yaw =yaw
        self.v = v
        self.rear_x = self.x
        self.rear_y = self.y
        print("here",self.x,self.y,self.v)

    def update(self, a, delta):
        self.x += self.v * math.cos(self.yaw) * dt
        self.y += self.v * math.sin(self.yaw) * dt
        self.yaw += self.v / WB * math.tan(delta) * dt
        self.v += a * dt
        self.rear_x = self.x - ((WB / 2) * math.cos(self.yaw))
        self.rear_y = self.y - ((WB / 2) * math.sin(self.yaw))

    def calc_distance(self, point_x, point_y):
        dx = self.rear_x - point_x
        dy = self.rear_y - point_y
        return math.hypot(dx, dy)
    def gps_callback(self,msg):
        self.x,self.y,__,__ = utm.from_latlon(msg.latitude,msg.longitude)
        r = R.from_quat([self.imu_msg.orientation.w,self.imu_msg.orientation.x,self.imu_msg.orientation.y,self.imu_msg.orientation.z])
        eulars = r.as_euler('zyx',degrees=False)
        print(eulars[1])
        self.yaw = eulars[1]
        self.v = msg.speed
        self.rear_x = self.x
        self.rear_y = self.y
        #print(msg.latitude)
    def imu_callback(self,msg):
        self.imu_msg= msg
        #print(msg.orientation.x)
    def publish(self,speed,delta):
        self.cmd_vel.linear.x = speed
        self.cmd_vel.angular.z = self.cmd_vel.angular.z + delta
        self.cmd_pub.publish(self.cmd_vel)

class States:

    def __init__(self):
        self.x = []
        self.y = []
        self.yaw = []
        self.v = []
        self.t = []

    def append(self, t, state):
        self.x.append(state.x)
        self.y.append(state.y)
        self.yaw.append(state.yaw)
        self.v.append(state.v)
        self.t.append(t)


def proportional_control(target, current):
    a = Kp * (target - current)

    return a

class TargetCourse:

    def __init__(self, cx, cy):
        self.cx = cx
        self.cy = cy
        self.old_nearest_point_index = None

    def search_target_index(self, state):

        # To speed up nearest point search, doing it at only first time.
        if self.old_nearest_point_index is None:
            # search nearest point index
            dx = [state.rear_x - icx for icx in self.cx]
            dy = [state.rear_y - icy for icy in self.cy]
            d = np.hypot(dx, dy)
            ind = np.argmin(d)
            self.old_nearest_point_index = ind
        else:
            ind = self.old_nearest_point_index
            distance_this_index = state.calc_distance(self.cx[ind],
                                                      self.cy[ind])
            while True:
                distance_next_index = state.calc_distance(self.cx[ind + 1],
                                                          self.cy[ind + 1])
                if distance_this_index < distance_next_index:
                    break
                ind = ind + 1 if (ind + 1) < len(self.cx) else ind
                distance_this_index = distance_next_index
            self.old_nearest_point_index = ind

        Lf = k * state.v + Lfc  # update look ahead distance

        # search look ahead target point index
        while Lf > state.calc_distance(self.cx[ind], self.cy[ind]):
            if (ind + 1) >= len(self.cx):
                break  # not exceed goal
            ind += 1

        return ind, Lf



def pure_pursuit_steer_control(state, trajectory, pind):
    ind, Lf = trajectory.search_target_index(state)

    if pind >= ind:
        ind = pind

    if ind < len(trajectory.cx):
        tx = trajectory.cx[ind]
        ty = trajectory.cy[ind]
    else:  # toward goal
        tx = trajectory.cx[-1]
        ty = trajectory.cy[-1]
        ind = len(trajectory.cx) - 1

    alpha = math.atan2(ty - state.rear_y, tx - state.rear_x) - state.yaw

    # using two line angle
    if two_line_angle == True:
      if ind+1<len(trajectory.cx):
         tx2 = trajectory.cx[ind+1]
         ty2 = trajectory.cy[ind+1]
      else:
         tx2=tx
         ty2=ty
      v_1 = [tx-state.rear_x,ty-state.rear_y]
      v_2 = [tx2-state.rear_x,ty2-state.rear_y]
      cross = np.cross(v_1, v_2)
      alpha = math.atan2(np.hypot(cross[1],cross[2]), np.dot(v_1, v_2)); # delta angle to be rotated



    # compute angle needs to be rotated 
    delta = math.atan2(2.0 * WB * math.sin(alpha) / Lf, 1.0)*(180/math.pi) # delta angle for wheel
    #delta= math.atan2(alpha*WB,state.v)

    return delta, ind


def plot_arrow(x, y, yaw, length=1.0, width=0.5, fc="r", ec="k"):
    """
    Plot arrow
    """

    if not isinstance(x, float):
        for ix, iy, iyaw in zip(x, y, yaw):
            plot_arrow(ix, iy, iyaw)
    else:
        plt.arrow(x, y, length * math.cos(yaw), length * math.sin(yaw),
                  fc=fc, ec=ec, head_width=width, head_length=width)
        plt.plot(x, y)

def get_straight_course(dl):
    #ax = [0.0, 5.0, 10.0, 20.0, 30.0, 40.0, 50.0]
    #ay = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]


    data = np.genfromtxt('/home/iisri/matlabCode/git_repo_new/simulinkObstacleAvoidance/curved_gps_imu_ref_john_deer.csv', delimiter=',')
    #data = np.genfromtxt('/home/iisri/matlabCode/git_repo_new/simulinkObstacleAvoidance/ref_gps+imu_johndeer.csv', delimiter=',')
    #print(data[:2,])
    ax,ay,__,__ = utm.from_latlon(data[4:,0],data[4:,1])
    #print(ax[1],ay[1])
    cx, cy, cyaw, ck, s = cubic_spline_planner.calc_spline_course(
        ax, ay, ds=dl)

    return cx, cy, cyaw, ck
def main():
    #  target course
    #cx = np.arange(0, 50, 0.5)
    #cy = [math.sin(ix / 5.0) * ix / 2.0 for ix in cx]

    data = np.genfromtxt('/home/iisri/matlabCode/git_repo_new/simulinkObstacleAvoidance/curved_gps_imu_ref_john_deer.csv', delimiter=',')
    #data = np.genfromtxt('/home/iisri/matlab_files/git_repo/simulinkObstacleAvoidance/ref_gps+imu_johndeer.csv', delimiter=',')
    #get initail yaw
    ax,ay,__,__ = utm.from_latlon(data[1:,0],data[1:,1])

    #print(ax,ay)
    d_ax = ax[0]-ax[2]
    d_ay = ay[0]-ay[2]
    init_yaw = math.atan2(d_ay,d_ax)
    # get the simulated path
    dl = 1.0  # course tick
    cx, cy, cyaw, ck = get_straight_course(dl)




    target_speed = 10.0 / 3.6  # [m/s]

    T = 100.0  # max simulation time

    # initial state
    state = State(x=ax[0], y=ay[0], yaw = init_yaw, v=0.0)

  #initial yaw compensation
    if state.yaw - cyaw[0] >= math.pi:
        state.yaw -= math.pi * 2.0
    elif state.yaw - cyaw[0] <= -math.pi:
        state.yaw += math.pi * 2.0

    lastIndex = len(cx) - 1
    time = 0.0
    states = States()
    print(state.x,state.y)
    states.append(time, state)
    target_course = TargetCourse(cx, cy)
    target_ind, _ = target_course.search_target_index(state)
    rospy.init_node("Follower")
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        rospy.Subscriber("/gps",GPSFix , state.gps_callback)
        rospy.Subscriber("/imu/data",Imu,state.imu_callback)

        #ai = proportional_control(target_speed, state.v)
        di, target_ind = pure_pursuit_steer_control(
                state, target_course, target_ind)
        state.publish(target_speed,di)
        states.append(time, state)
        if show_animation:  # pragma: no cover
            plt.cla()
            # for stopping simulation with the esc key.
            plt.gcf().canvas.mpl_connect(
                'key_release_event',
                lambda event: [exit(0) if event.key == 'escape' else None])
            plot_arrow(state.x, state.y, state.yaw)
            plt.plot(cx, cy, "-r", label="course")
            plt.plot(states.x, states.y, "-b", label="trajectory")
            plt.plot(cx[target_ind], cy[target_ind], "xg", label="target")
            plt.axis("equal")
            plt.grid(True)
            plt.title("Speed[km/h]:" + str(state.v * 3.6)[:4])
            plt.pause(0.001)
        # Test
        assert lastIndex >= target_ind, "Cannot goal"

        if show_animation:  # pragma: no cover
            plt.cla()
            plt.plot(cx, cy, ".r", label="course")
            plt.plot(states.x, states.y, "-b", label="trajectory")
            plt.legend()
            plt.xlabel("x[m]")
            plt.ylabel("y[m]")
            plt.axis("equal")
            plt.grid(True)

            plt.subplots(1)
            plt.plot(states.t, [iv * 3.6 for iv in states.v], "-r")
            plt.xlabel("Time[s]")
            plt.ylabel("Speed[km/h]")
            plt.grid(True)
            plt.show()
        rate.sleep()




if __name__ == '__main__':
    print("Pure pursuit path tracking simulation start")
    main()
