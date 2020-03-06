import math
'''Receiving data'''
x_des = GPS_IMU_DATA[0]
y_des = GPS_IMU_DATA[1]
theta_des = GPS_IMU_DATA[2]
v_des = GPS_IMU_DATA[3]
x = GPS_IMU_DATA[4]
y = GPS_IMU_DATA[5]
theta = GPS_IMU_DATA[6]
v = GPS_IMU_DATA[7]

'''Vehicle's parameters'''
length = 1 # length of the vehicle
steerting = 1 # steering ratio

'''Errors'''
ex = x - x_des
ey = y - y_des
etheta = theta - math.atan2(-y , -x)
ev = v - v_des

'''Controls'''
a = -ev;
if v != 0:
    etheta = theta - math.real(math.acos((-5*ex)/v)) - math.real(math.asin((-5*ey)/v))
    delta = math.atan2(- etheta * length , v)
else:
    delta = 0

'''Dynamics'''
dx = math.cos(theta) * v
dy = math.sin(theta) * v
dtheta = math.tan(delta) * v / length
dv = steerting * a
