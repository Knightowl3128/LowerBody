from Robot import *
from numpy import *
import matplotlib.pyplot as plt
from trajectory import traj

fig = plt.figure(1)
ax = fig.add_subplot(111, projection='3d')
# creating the robot
body = Robot()
speed = 0.01
body.time_step = speed
# creating the core and the corresponding links
# name, id, mass, direction, mother, child)
core_l = Link('Centre', 0, 0.18520035953947, 'Left', None, 1)
l_1 = Link('Hip_1', 1, .0843835237958419, 'Left', 0, 2)  # zero mass as there is no physical link (ball joint)
l_2 = Link('Hip_2', 2, 0.0831707898707576, 'Left', 1, 3)
l_3 = Link('Thigh', 3, 0.116386255838466, 'Left', 2, 4)
l_4 = Link('Knee', 4, 0.115582219792398, 'Left', 3, 5)
l_5 = Link('Ankle', 5, 0.0467879950319091, 'Left', 4, None)

core_r = Link('Centre', 0, 0.18520035953947, 'Right', None, 1)
r_1 = Link('Hip_1', 1, .0843835237958419, 'Right', 0, 2)
r_2 = Link('Hip_2', 2, 0.0831707898707576, 'Right', 1, 3)
r_3 = Link('Thigh', 3, 0.116386255838466, 'Right', 2, 4)
r_4 = Link('Knee', 4, 0.115582219792398, 'Right', 3, 5)
r_5 = Link('Ankle', 5, 0.0467879950319091, 'Right', 4, None)

body.add_link(core_l)
body.add_link(l_1)
body.add_link(l_2)
body.add_link(l_3)
body.add_link(l_4)
body.add_link(l_5)

body.add_link(core_r)
body.add_link(r_1)
body.add_link(r_2)
body.add_link(r_3)
body.add_link(r_4)
body.add_link(r_5)

# d-h parameters
core_l.dh = array([[0.0, 0, 1.0, 0]])
l_1.dh = array([[0, 0, 0, pi / 2]])
l_2.dh = array([[0, 0, 3, pi / 2]])
l_3.dh = array([[0, 0, 9.0, 0]])
l_4.dh = array([[0, 0, 11.0, 0]])
l_5.dh = array([[0.0, 0, 1, 0]])

core_r.dh = array([[0.0, 0, -1, 0]])
r_1.dh = array([[0.0, 0, 0, pi / 2]])
r_2.dh = array([[0.0, 0, 3, pi / 2]])
r_3.dh = array([[0.0, 0, 9, 0]])
r_4.dh = array([[0.0, 0, 11, 0]])
r_5.dh = array([[0.0, 0, 1, 0]])

# inertia tensors

# for l_1 and core it is zero as it never rotates
l_2.I = (array([[2.94087635631147E-05, -1.19904545841814E-08, -2.40263672865729E-07],
                [-1.19904545841814E-08, 3.59801748266375E-05, 7.9755837503152E-07],
                [-2.40263672865729E-07, 7.9755837503152E-07, 2.29200602273928E-05]]))
l_3.I = (array([[0.000357762131890894, -4.71321103204108E-05, 2.18885095933642E-06],
                [-4.71321103204108E-05, 5.14619739065129E-05, -5.54069244270054E-06],
                [2.18885095933642E-06, -5.54069244270054E-06, 0.000370171570432626]]))
l_4.I = (array([[0.000397225397723862, -1.73262609639968E-05, 1.32921239310872E-08],
                [-1.73262609639968E-05, 3.98086710206889E-05, -5.89721905441721E-07],
                [1.32921239310872E-08, -5.89721905441721E-07, 0.000393657905933943]]))
# l_5.I = (l_3.mass*array([[1**2+0.5**2,0,0],[0, 11**2+0.5**2,0],[0, 0, 11**2+1**2]]))/12

r_2.I = (array([[2.94087635631147E-05, -1.19904545841814E-08, -2.40263672865729E-07],
                [-1.19904545841814E-08, 3.59801748266375E-05, 7.9755837503152E-07],
                [-2.40263672865729E-07, 7.9755837503152E-07, 2.29200602273928E-05]]))
r_3.I = (array([[0.000357762131890894, -4.71321103204108E-05, 2.18885095933642E-06],
                [-4.71321103204108E-05, 5.14619739065129E-05, -5.54069244270054E-06],
                [2.18885095933642E-06, -5.54069244270054E-06, 0.000370171570432626]]))
r_4.I = (array([[0.000397225397723862, -1.73262609639968E-05, 1.32921239310872E-08],
                [-1.73262609639968E-05, 3.98086710206889E-05, -5.89721905441721E-07],
                [1.32921239310872E-08, -5.89721905441721E-07, 0.000393657905933943]]))

body.set_angle([0, 0, -pi / 2, 0.3522279392980471, -0.6383816108049025], 'Left')
body.set_angle([0, 0, -pi / 2, 0.3522279392980471, -0.6383816108049025], 'Right')
speed = 4
j = 0

iteration = 0

t = 0
fs = 0
footPosL = [1, 0, 0]
footPosR = [-1, 0, 0]

switch_timer = 0
left_l = False
angles_l = body.inverse_kinematics([1, 0, 0], 'Left')
angles_r = body.inverse_kinematics([-1, 0, 0], 'Right')

foot_height = 1
step_size = 6

X = [0, 0, 0]
Y = [0, 0, 0]
Xc = [0, 0, 0]
Yc = [0, 0, 0]

while True:

    ax.axes.set_xlim3d(left=-10, right=10)
    ax.axes.set_ylim3d(bottom=-40, top=40 - speed)
    ax.axes.set_zlim3d(bottom=0, top=40)
    # body.CoM = array([[a[2], -a[1], 23]])
    body.CoM = array([[-traj(t)[2], -traj(t)[1], 23]])
    #    l1 -= l*0.1*speed*( sign( sin(j*pi) )      + sign( sin(j*pi +   pi/2) ) + sign( sin(j*2*pi) ) + 1 )
    #    l2 -= l*0.1*speed*( sign( sin(j*pi + pi) ) + sign( sin(j*pi + 1.5*pi) ) + sign( sin(j*2*pi + 2*pi) ) + 1 )

    # body.CoM = array([[0, 0, 23]])
    # y1 = (sin(j * pi)) * (sign(sin(j * 2 * pi * 0.5)) + sign(sin(j * 2 * pi * 0.5 + pi / 2)) + sign(
    #     sin(j * 2 * pi)) + 1) * 0.5
    #
    # y2 = (sin(j * 2 * pi)) * (
    #     (1 + sign(sin(j * 2 * pi)) + sign(sin(j * 2 * pi * 0.5 + pi)) + sign(sin(j * 2 * pi * 0.5 - pi / 2)))) * 0.5
    if t == 0:
        spline_h = CubicSpline([0, 1, 2], [0, foot_height, 0], bc_type=(((1, 0)), (1, 0)))
        spline_y = CubicSpline([0, 2], [0, step_size], bc_type=(((1, 0)), (1, 0)))
    if t <= 2 and t != 0:
        angles_l = body.inverse_kinematics([1, -spline_y(t), spline_h(t)], 'Left')
        angles_r = body.inverse_kinematics([-1, 0, 0], 'Right')

    if not t <= 2:
        if abs(round(switch_timer, 5)) <= 0.0001:
            if left_l:
                spline_h_l = CubicSpline([0, 1, 2], [0, foot_height, 0], bc_type=(((1, 0)), (1, 0)))
                spline_y_l = CubicSpline([0, 2], [body.links_l[5].end[0, 1], -12 + body.links_l[5].end[0, 1]],
                                         bc_type=(((1, 0)), (1, 0)))
                swing_leg = 'Left'
                switch_timer = 2.0 + speed
            if not left_l:
                spline_h_r = CubicSpline([0, 1, 2], [0, foot_height, 0], bc_type=(((1, 0)), (1, 0)))
                spline_y_r = CubicSpline([0, 2], [body.links_r[5].end[0, 1], -12 + body.links_r[5].end[0, 1]],
                                         bc_type=(((1, 0)), (1, 0)))
                swing_leg = 'Right'
                switch_timer = 2.0 + speed
            left_l = not left_l


        elif switch_timer >= 0.0001:
            switch_timer -= speed

            if swing_leg == 'Left':
                k = 2 - switch_timer
                angles_l = body.inverse_kinematics([1, spline_y_l(k), spline_h_l(k)], 'Left')

                angles_r = body.inverse_kinematics([-1, r_5.end[0, 1], 0], 'Right')
            elif swing_leg == 'Right':
                k = 2 - switch_timer
                angles_r = body.inverse_kinematics([-1, spline_y_r(k), spline_h_r(k)], 'Right')
                angles_l = body.inverse_kinematics([1, l_5.end[0, 1], 0], 'Left')

    # angles_l = body.inverse_kinematics([-1, 0, 0], 'Left')
    # angles_r = body.inverse_kinematics([-1, 0, 0], 'Right')
    body.set_angle(angles_l, 'Left')
    body.set_angle(angles_r, 'Right')
    body.get_all_pos()
    t += speed

    if iteration > 2:
        x_zmp, y_zmp = body.find_zmp()
        X.append(-x_zmp)
        Y.append(-y_zmp)
        Xc.append(-body.find_CoM_Pos()[0][0, 0])
        Yc.append(-body.find_CoM_Pos()[0][0, 1])
        ax.scatter(x_zmp, y_zmp, 0)
        # plt.figure(2)
        # ax1 = plt.subplot(211)
        # ax2 = plt.subplot(212)
        # #
        # ax1.scatter(t, x_zmp, c='green', s=10)
        # ax2.scatter(t, y_zmp, c='red', s=10)
    iteration += 1
    if iteration == 50:
        break

    body.draw_all(ax)

    ax.scatter(body.find_CoM_Pos()[0][0, 0], body.find_CoM_Pos()[0][0, 1], body.find_CoM_Pos()[0][0, 2])
    ax.grid()

    plt.pause(0.000000000000000001)
    ax.cla()

plt.figure(2)

time = arange(0, t, speed)
plt.plot(time, Y)
plt.plot(time, Yc)
plt.show()
