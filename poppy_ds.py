#!/usr/bin/env python3
from Robot import *
from numpy import *
import matplotlib.pyplot as plt
from LIPM_with_dsupport import *
import rospy
from std_msgs.msg import Float64
import subprocess
from mono_define import *

rospy.init_node('mono_move')
fig = plt.figure(1)
ax = fig.add_subplot(111, projection='3d')

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

# rospy.Subscriber("chatter", String, callback)

if not show:

    initiate_time = 5
    speed = 0.01
    angles_l = [0, 0, -pi / 2, 0, 0, pi / 2]
    angles_r = [0, 0, -pi / 2, 0, 0, pi / 2]
    body.set_angle(angles_l, 'Left')
    body.set_angle(angles_r, 'Right')
    body.get_all_pos()

    s = subprocess.check_call("rosservice call /gazebo/reset_simulation \"{}\"", shell=True)
    rospy.sleep(0.1)
    body.ros_publish()
    s = subprocess.check_call("rosservice call /gazebo/reset_simulation \"{}\"", shell=True)
    rospy.sleep(0.4)

    r = subprocess.check_call("rosservice call gazebo/unpause_physics", shell=True)
    rospy.sleep(0.4)

else:
    initiate_time = 5
    speed = 0.1
rate = rospy.Rate(1 / speed)
#####initiate##########
# spline_1, spline_2, spline_3 = body.transition_angle([-pi / 2, 0, 0],
#                         [-1.5707963267948966, 0.5610612566993562, -1.0000099541506042],initiate_time)

initial_height = 0.70

body.CoM = array([[0.015 - 0.09, 0, initial_height]])
spline_1, spline_2, spline_3 = body.transition_angle([-pi / 2, 0, 0],
                                                     body.inverse_kinematics([0.09, 0, 0], "Left")[2:],
                                                     initiate_time)

# spline_1, spline_2, spline_3 = body.transition_angle([-pi / 2, 0, 0],
#                                                      [-1.4204248987877621, 0.5362958375469041, -0.9566801401928506],
#                                                      initiate_time)
msg = Float64()
# bending the knees
t = 0
while t <= initiate_time and show == False:
    print(1)
    angles_l = [0, 0, spline_1(t), spline_2(t), spline_3(t), pi / 2]
    angles_r = [0, 0, spline_1(t), spline_2(t), spline_3(t), pi / 2]

    body.set_angle(angles_l, 'Left')
    body.set_angle(angles_r, 'Right')
    body.get_all_pos()
    body.ros_publish()

    t += speed
    rate.sleep()

t = 0
foot_height = 0.08
step_size = .1
iteration = 0
switch_timer = 0
left_l = False
foot_origin_ds = 0.09

foot_last_pos = [0, 0]

body.CoM = array([[0.015 - 0.09, 0, initial_height]])
l_6.end = array([[0.09, 0, 0]])
r_6.end = array([[-0.09, 0, 0]])
# these are the best results initiate_time = 0.65 T_dbl = 0.1 zc = 0.6
initiate_time = 0.7
T_dbl = 0.09
speed = 0.01
zc = 0.5
# initiate_time = 0.43
# T_dbl = 0.09
# speed = 0.01
# zc = 0.36
xsolve, vxsolve, ysolve, vysolve, p_mod = LIPM(speed, initiate_time, T_dbl, zc)
body.time_step = speed
rate = rospy.Rate(1 / speed)

print(body.inverse_kinematics([0.09, 0, 0], 'Left'))
while not rospy.is_shutdown():
    body.ros_subscribe()
    print(l_5.process_value)
    body.CoM = array([[ysolve[iteration] - 0.09, -xsolve[iteration], initial_height]])

    if abs(round(switch_timer, 3)) == 0:
        if t == 0:
            step_multi = 0  # first step takes only 1 step size but the second step covers twice the d/s
            second_step = True
        else:
            step_multi = 2

        if second_step == True and t != 0:
            second_step = False
            step_multi = 1

        if left_l:
            spline_h_l = CubicSpline([0, initiate_time / 2, initiate_time], [0, foot_height, 0],
                                     bc_type=(((1, 0)), (1, 0)))
            spline_y_l = CubicSpline([0, initiate_time],
                                     [body.links_l[6].end[0, 1], -step_multi * step_size + body.links_l[6].end[0, 1]],
                                     bc_type=(((1, 0)), (1, 0)))
            swing_leg = 'Left'
            switch_timer = initiate_time + T_dbl
            ds_timer = T_dbl
            dbl_phase = False
            foot_last_pos[0] = r_6.end[0, 0]
            foot_last_pos[1] = r_6.end[0, 1]
            angles_r = body.inverse_kinematics([foot_last_pos[0], foot_last_pos[1], 0], 'Right')

            angles_l = body.inverse_kinematics([foot_origin_ds, spline_y_l(0), spline_h_l(0)], 'Left')
            # k = speed
        if not left_l:
            spline_h_r = CubicSpline([0, initiate_time / 2, initiate_time], [0, foot_height, 0],
                                     bc_type=((1, 0), (1, 0)))
            spline_y_r = CubicSpline([0, initiate_time],
                                     [body.links_r[6].end[0, 1], -step_multi * step_size + body.links_r[6].end[0, 1]],
                                     bc_type=((1, 0), (1, 0)))
            swing_leg = 'Right'
            switch_timer = initiate_time + T_dbl
            ds_timer = T_dbl
            dbl_phase = False
            # k = speed
            foot_last_pos[0] = l_6.end[0, 0]
            foot_last_pos[1] = l_6.end[0, 1]
            angles_l = body.inverse_kinematics([foot_last_pos[0], foot_last_pos[1], 0], 'Left')
            angles_r = body.inverse_kinematics([-foot_origin_ds, spline_y_r(0), spline_h_r(0)], 'Right')
            # print(foot_last_pos)
        left_l = not left_l


    elif abs(round(switch_timer, 4)) > 0:
        switch_timer -= speed

        if swing_leg == 'Left':
            k = initiate_time + T_dbl - switch_timer
            # k +=speed
            if round(k, 2) == initiate_time:
                dbl_phase = True
            if dbl_phase == True:
                k = initiate_time
                print(k, body.CoM + 0.09)

            if abs(round(switch_timer, 4)) == 0:
                switch_timer = 0
                t += speed
                continue
            angles_l = body.inverse_kinematics([foot_origin_ds, spline_y_l(k), spline_h_l(k)], 'Left')
            angles_r = body.inverse_kinematics([foot_last_pos[0], foot_last_pos[1], 0], 'Right')
            # print("this is in main", end=" ")
            # print(rad2deg(angles_l))

        elif swing_leg == 'Right':
            k = initiate_time + T_dbl - switch_timer
            # k+=speed
            if round(k, 2) == initiate_time:
                dbl_phase = True

            if dbl_phase == True:
                k = initiate_time

            if abs(round(switch_timer, 4)) == 0:
                switch_timer = 0
                t += speed
                continue
            angles_r = body.inverse_kinematics([-foot_origin_ds, spline_y_r(k), spline_h_r(k)], 'Right')
            angles_l = body.inverse_kinematics([foot_last_pos[0], foot_last_pos[1], 0], 'Left')
            # print("this is in main after hit", end=" ")
            # print(rad2deg(angles_l))
    body.set_angle(angles_l, 'Left')
    body.set_angle(angles_r, 'Right')
    body.get_all_pos()
    body.ros_publish()
    iteration += 1
    rate.sleep()

# plt.plot(time,Y)
# plt.plot(time,traj(time)[1])
# plt.show()
