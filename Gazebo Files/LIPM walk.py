from Robot import *
from numpy import *
import matplotlib.pyplot as plt
from mono_define import *

fig = plt.figure(1)
ax = fig.add_subplot(111, projection='3d')

body.set_angle([0, 0, -1.5707963267948966, 0.3562144559334822, -0.63848537302278], 'Left')
body.set_angle([0, 0, -1.5707963267948966, 0.3562144559334822, -0.63848537302278], 'Right')
com_h = 0.5
body.CoM = array([[0, 00, com_h]])
body.set_angle(body.inverse_kinematics([-0.09, 0, 0.1], 'Right'), 'Right')
t = 0
iteration = 0
Vc = []
k = 0
speed = 0.009
body.time_step = speed

Tc = (0.31 / 9.81) ** 0.5
while True:
    ax.axes.set_xlim3d(left=-0.35, right=0.35)
    ax.axes.set_ylim3d(bottom=-1, top=1)
    ax.axes.set_zlim3d(bottom=0, top=.8)
    # body.CoM = array([[0.1*sin(0.5*t), 0.1*sin(0.5*t), 0.5]])
    px = (0.0 - 0.09) * cosh(t / Tc) + 0.09
    py = (-0.2 - 0.0) * cosh(t / Tc) + 0.0
    body.CoM = array([[px, py, com_h]])

    body.set_angle(body.inverse_kinematics([0.09, 0, 0], 'Left'), 'Left')
    # body.set_angle(body.inverse_kinematics([-0.09,0,0],'Right'), 'Right')
    l_6.q = pi / 2
    body.get_all_pos()
    t += speed
    body.draw_all(ax)
    ax.scatter(body.find_CoM_Pos()[0][0, 0], body.find_CoM_Pos()[0][0, 1], body.find_CoM_Pos()[0][0, 2])
    ax.grid()
    x_zmp, y_zmp = body.find_zmp()
    Vc.append((body.find_CoM_Pos()[0][0, 0] - k) / 0.1)
    k = body.find_CoM_Pos()[0][0, 0]
    ax.scatter(x_zmp, y_zmp, 0)
    ax.scatter(body.find_CoM_Pos()[0][0, 0], body.find_CoM_Pos()[0][0, 1], 0, color='red')
    print(body.find_CoM_Pos()[0])
    # if iteration>=50:
    #     break
    iteration += 1
    # print('l_6',end="")
    # print('l_5', end="")
    # print(l_5.end)
    # print('-------------------------------')
    plt.pause(0.000000000000000001)
    ax.cla()
plt.figure(2)

time = linspace(0, 0.1 * 51, 51)
# plt.plot(time, X)
plt.plot(time, Vc)
plt.show()
