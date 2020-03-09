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

# Parameters
k = 1  # look forward gain
Lfc = 1.0  # [m] look-ahead distance
Kp = 1.0  # speed proportional gain
dt = 0.1  # [s] time tick
WB = 1.2 # [m] wheel base of vehicle

two_line_angle = False
parham_pp = False
show_animation = True
stanely = False
max_steer = np.radians(60.0)  # [rad] max steering angle
goal_dis = 2
sys.path.append("../../PathPlanning/CubicSpline/")

try:
    import cubic_spline_planner
except:
    raise

class State:

    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v
        self.rear_x = self.x - ((WB / 2) * math.cos(self.yaw))
        self.rear_y = self.y - ((WB / 2) * math.sin(self.yaw))
        #print(self.x,self.y)
    def update(self, a, delta):
        if stanely == True:
           delta = np.clip(delta, -max_steer, max_steer)
        self.x += self.v * math.cos(self.yaw) * dt
        self.y += self.v * math.sin(self.yaw) * dt
        self.yaw += self.v / WB * math.tan(delta) * dt
        if stanely == True:
           self.yaw = normalize_angle(self.yaw)
        self.v += a * dt
        self.rear_x = self.x - ((WB / 2) * math.cos(self.yaw))
        self.rear_y = self.y - ((WB / 2) * math.sin(self.yaw))

    def calc_distance(self, point_x, point_y):
        dx = self.rear_x - point_x
        dy = self.rear_y - point_y
        return math.hypot(dx, dy)


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
                #if (ind + 1) < len(self.cx):
                #    break
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

      m1 =(ty-state.rear_y)/(tx-state.rear_x)
      m2 =(ty2-ty)/(tx2-tx)

      alpha = math.atan2(abs(m1-m2)/(1+m1*m2),1)
    elif parham_pp == True:
      ex = state.rear_x - tx
      ey = state.rear_y - ty
      #etheta = sate.ya - math.atan2(-y , -x)
      ev = state.v - (10.0 / 3.6)
      etheta = state.yaw - (np.arccos((-5*ex)/state.v).real) - (np.arcsin((-5*ey)/state.v).real)




      # v_1 = [tx-state.rear_x,ty-state.rear_y]
      # v_2 = [tx2-state.rear_x,ty2-state.rear_y]
      # cross = np.cross(v_1, v_2)
      # print('cross:',cross)
      # alpha = math.atan2(np.sqrt(cross**2), np.dot(v_1, v_2)) # delta angle to be rotated

    # delta angle to be rotated
    if parham_pp==True:
        delta = math.atan2(- etheta * WB, state.v)
    else:
        delta = math.atan2(2.0 * WB * math.sin(alpha) / Lf, 1.0)

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


    data = np.genfromtxt('/home/iisri/matlabCode/git_repo_new/simulinkObstacleAvoidance/09032020straight.csv', delimiter=',')
    #data = np.genfromtxt('/home/iisri/matlab_files/git_repo/simulinkObstacleAvoidance/ref_gps+imu_johndeer.csv', delimiter=',')
    #print(data[:2,])
    ax,ay,__,__ = utm.from_latlon(data[4:,0],data[4:,1])
    #print(ax[1],ay[1])
    cx, cy, cyaw, ck, s = cubic_spline_planner.calc_spline_course(
        ax, ay, ds=dl)

    return cx, cy, cyaw, ck
def stanley_control(state, cx, cy, cyaw, last_target_idx):
    """
    Stanley steering control.

    :param state: (State object)
    :param cx: ([float])
    :param cy: ([float])
    :param cyaw: ([float])
    :param last_target_idx: (int)
    :return: (float, int)
    """
    current_target_idx, error_front_axle = calc_target_index(state, cx, cy)

    if last_target_idx >= current_target_idx:
        current_target_idx = last_target_idx

    # theta_e corrects the heading error
    theta_e = normalize_angle(cyaw[current_target_idx] - state.yaw)
    # theta_d corrects the cross track error
    theta_d = np.arctan2(k * error_front_axle, state.v)
    # Steering control
    delta = theta_e + theta_d

    return delta, current_target_idx
def normalize_angle(angle):
    """
    Normalize an angle to [-pi, pi].

    :param angle: (float)
    :return: (float) Angle in radian in [-pi, pi]
    """
    while angle > np.pi:
        angle -= 2.0 * np.pi

    while angle < -np.pi:
        angle += 2.0 * np.pi

    return angle

def calc_target_index(state, cx, cy):
    """
    Compute index in the trajectory list of the target.

    :param state: (State object)
    :param cx: [float]
    :param cy: [float]
    :return: (int, float)
    """
    # Calc front axle position
    fx = state.x + WB * np.cos(state.yaw)
    fy = state.y + WB * np.sin(state.yaw)

    # Search nearest point index
    dx = [fx - icx for icx in cx]
    dy = [fy - icy for icy in cy]
    d = np.hypot(dx, dy)
    target_idx = np.argmin(d)

    # Project RMS error onto front axle vector
    front_axle_vec = [-np.cos(state.yaw + np.pi / 2),
                      -np.sin(state.yaw + np.pi / 2)]
    error_front_axle = np.dot([dx[target_idx], dy[target_idx]], front_axle_vec)

    return target_idx, error_front_axle

def main():
    #  target course
    #cx = np.arange(0, 50, 0.5)
    #cy = [math.sin(ix / 5.0) * ix / 2.0 for ix in cx]

    data = np.genfromtxt('/home/iisri/matlabCode/git_repo_new/simulinkObstacleAvoidance/09032020straight.csv', delimiter=',')
    #data = np.genfromtxt('/home/iisri/matlab_files/git_repo/simulinkObstacleAvoidance/ref_gps+imu_johndeer.csv', delimiter=',')
    #get initail yaw
    ax,ay,__,__ = utm.from_latlon(data[1:,0],data[1:,1])

    #print(ax,ay)
    d_ax = ax[0]-ax[3]
    d_ay = ay[0]-ay[3]
    init_yaw = math.atan2(d_ay,d_ax)
    # get the simulated path
    dl = 0.1  # course tick
    cx, cy, cyaw, ck = get_straight_course(dl)




    target_speed =5.0 / 3.6  # [m/s]

    T = 100.0  # max simulation time

    # initial state
    state = State(x=ax[0], y=ay[0], yaw=0.0, v=0.0)

  #initial yaw compensation
    if state.yaw - cyaw[0] >= math.pi:
        state.yaw -= math.pi * 2.0
    elif state.yaw - cyaw[0] <= -math.pi:
        state.yaw += math.pi * 2.0

    lastIndex = len(cx) - 1
    time = 0.0
    states = States()
    states.append(time, state)
    target_course = TargetCourse(cx, cy)
    target_ind, _ = target_course.search_target_index(state)

    if stanely == True:
       target_idx, _ = calc_target_index(state, cx, cy)

    while T >= time and lastIndex > target_ind:

        # Calc control input
        ai = proportional_control(target_speed, state.v)

        if stanely == True:
           di, target_idx = stanley_control(state, cx, cy, cyaw, target_idx)
        else:
           di, target_ind = pure_pursuit_steer_control(state, target_course, target_ind)



        state.update(ai, di)  # Control vehicle

        time += dt
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


if __name__ == '__main__':
    print("Pure pursuit path tracking simulation start")
    main()
