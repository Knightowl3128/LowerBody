from Robot import *
from numpy import *
import matplotlib.pyplot as plt
from mono_define import *

fig = plt.figure(1)
ax = fig.add_subplot(111, projection='3d')
speed = 0.05
body.set_angle([0, 0, -1.5707963267948966, 0.3562144559334822, -0.63848537302278], 'Left')
body.set_angle([0, 0, -1.5707963267948966, 0.3562144559334822, -0.63848537302278], 'Right')
body.CoM = array([[0.1, 00, 0.64]])
body.time_step = speed

t = 0
iteration = 0
Vc = []
bh = body.CoM
v = body.CoM
k = 0
while True:
    ax.axes.set_xlim3d(left=-1, right=1)
    ax.axes.set_ylim3d(bottom=-1, top=1)
    ax.axes.set_zlim3d(bottom=0, top=.8)
    body.CoM = array([[0.07 * cos(3 * t), 0, .6]])

    v = (body.find_CoM_Pos()[0][0, 0] - bh) / speed
    bh = body.find_CoM_Pos()[0][0, 0]
    print(body.inverse_kinematics([0.09, 0, 0], 'Left'))
    print(l_6.end)

    body.set_angle(body.inverse_kinematics([0.09, 0, 0], 'Left'), 'Left')
    body.set_angle(body.inverse_kinematics([-0.09, 0, 0], 'Right'), 'Right')
    # body.set_angle([0, 0, -1.4204248987877621, 0.5362958375469041, -0.9566801401928506], 'Left')
    # body.set_angle([0, 0, -1.4204248987877621, 0.5362958375469041, -0.9566801401928506], 'Right')
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
