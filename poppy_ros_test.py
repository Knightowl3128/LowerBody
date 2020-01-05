#!/usr/bin/env python3
from Robot import *
from numpy import *
import matplotlib.pyplot as plt
from LIPM import *
import rospy
from std_msgs.msg import Float64
import subprocess
from mono_define import *

rospy.init_node('mono_move')
fig = plt.figure(1)
ax = fig.add_subplot(111, projection='3d')

foot_height = 0.05
step_size = .1

t = 0
iteration = 0
switch_timer = 0
left_l = False
switch_timer = 0
foot_origin_ds = 0.09

show = False

l_2.pub = rospy.Publisher('/mono/l_hip_roll_position/command', Float64, queue_size=1)
l_3.pub = rospy.Publisher('/mono/l_hip_pitch_position/command', Float64, queue_size=1)
l_4.pub = rospy.Publisher('/mono/l_knee_pitch_position/command', Float64, queue_size=1)
l_5.pub = rospy.Publisher('/mono/l_ankle_pitch_position/command', Float64, queue_size=1)
l_6.pub = rospy.Publisher('/mono/l_ankle_roll_position/command', Float64, queue_size=1)

r_2.pub = rospy.Publisher('/mono/r_hip_roll_position/command', Float64, queue_size=1)
r_3.pub = rospy.Publisher('/mono/r_hip_pitch_position/command', Float64, queue_size=1)
r_4.pub = rospy.Publisher('/mono/r_knee_pitch_position/command', Float64, queue_size=1)
r_5.pub = rospy.Publisher('/mono/r_ankle_pitch_position/command', Float64, queue_size=1)
r_6.pub = rospy.Publisher('/mono/r_ankle_roll_position/command', Float64, queue_size=1)

if not show:
    initiate_time = 10
    speed = 0.01
    s = subprocess.check_call("rosservice call /gazebo/reset_simulation \"{}\"", shell=True)
    rospy.sleep(1)
    r = subprocess.check_call("rosservice call gazebo/unpause_physics", shell=True)
    rospy.sleep(1)

else:
    initiate_time = 2
    speed = 0.1
rate = rospy.Rate(100)
#####initiate##########
spline_1, spline_2, spline_3 = body.transition_angle([-pi / 2, 0, 0],
                                                     [-1.5707963267948966, 0.5610612566993562, -1.0000099541506042],
                                                     initiate_time)

msg = Float64()
while t <= initiate_time and show == False:

    angles_l = [0, 0, spline_1(t), spline_2(t), spline_3(t), pi / 2]
    angles_r = [0, 0, spline_1(t), spline_2(t), spline_3(t), pi / 2]
    body.set_angle(angles_l, 'Left')
    body.set_angle(angles_r, 'Right')
    body.get_all_pos()
    body.ros_publish()

    t += speed
    rate.sleep()
    if show:
        ax.axes.set_xlim3d(left=-1, right=1)
        ax.axes.set_ylim3d(bottom=-1, top=1)
        ax.axes.set_zlim3d(bottom=0, top=.8)
        body.draw_all(ax)
        ax.grid()

        plt.pause(0.000000000000000001)
        ax.cla()
t = 0
foot_last_pos = [0, 0]
xsolve, vxsolve, ysolve, vysolve, p_mod = LIPM(speed, 2)

while not rospy.is_shutdown():
    body.CoM = array([[0.1 * sin(t), 0, 0.6]])
    print(body.CoM)
    body.set_angle(body.inverse_kinematics([0.09, 0, 0], 'Left'), 'Left')
    body.set_angle(body.inverse_kinematics([-0.09, 0, 0], 'Right'), 'Right')
    print(body.inverse_kinematics([0.09, 0, 0], 'Left'))
    body.get_all_pos()
    body.ros_publish()
    t += speed
    iteration += 1
    rate.sleep()

    if show:
        ax.axes.set_xlim3d(left=-1, right=1)
        ax.axes.set_ylim3d(bottom=-1, top=1)
        ax.axes.set_zlim3d(bottom=0, top=.8)
        body.draw_all(ax)
        ax.grid()

        plt.pause(0.000000000000000001)
        ax.cla()

# plt.plot(time,Y)
# plt.plot(time,traj(time)[1])
# plt.show()
