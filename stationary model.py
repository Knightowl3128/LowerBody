from Robot import *
from numpy import *
import matplotlib.pyplot as plt
from trajectory import traj

fig = plt.figure(1)
ax = fig.add_subplot(111, projection='3d')
# creating the robot
body = Robot()

# creating the core and the corresponding links
# name, id, mass, direction, mother, child)
core_l = Link('Centre', 0, 5, 'Left', None, 1)
l_1 = Link('Hip_1', 1, 0, 'Left', 0, 2)  # zero mass as there is no physical link (ball joint)
l_2 = Link('Hip_2', 2, 1, 'Left', 1, 3)
l_3 = Link('Thigh', 3, 1, 'Left', 2, 4)
l_4 = Link('Knee', 4, 1, 'Left', 3, 5)
l_5 = Link('Ankle', 5, 0, 'Left', 4, None)

core_r = Link('Centre', 0, 5, 'Right', None, 1)
r_1 = Link('Hip_1', 1, 0, 'Right', 0, 2)
r_2 = Link('Hip_2', 2, 1, 'Right', 1, 3)
r_3 = Link('Thigh', 3, 1, 'Right', 2, 4)
r_4 = Link('Knee', 4, 1, 'Right', 3, 5)
r_5 = Link('Ankle', 5, 0, 'Right', 4, None)

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
l_2.I = (l_2.mass * array([[1 ** 2 + 0.5 ** 2, 0, 0], [0, 3 ** 2 + 1 ** 2, 0], [0, 0, 3 ** 2 + 0.5 ** 2]])) / 12
l_3.I = (l_3.mass * array([[1 ** 2 + 0.5 ** 2, 0, 0], [0, 9 ** 2 + 0.5 ** 2, 0], [0, 0, 9 ** 2 + 1 ** 2]])) / 12
l_4.I = (l_3.mass * array([[1 ** 2 + 0.5 ** 2, 0, 0], [0, 11 ** 2 + 0.5 ** 2, 0], [0, 0, 11 ** 2 + 1 ** 2]])) / 12
# l_5.I = (l_3.mass*array([[1**2+0.5**2,0,0],[0, 11**2+0.5**2,0],[0, 0, 11**2+1**2]]))/12

r_2.I = (l_2.mass * array([[1 ** 2 + 0.5 ** 2, 0, 0], [0, 3 ** 2 + 1 ** 2, 0], [0, 0, 3 ** 2 + 0.5 ** 2]])) / 12
r_3.I = (l_3.mass * array([[1 ** 2 + 0.5 ** 2, 0, 0], [0, 9 ** 2 + 0.5 ** 2, 0], [0, 0, 9 ** 2 + 1 ** 2]])) / 12
r_4.I = (l_3.mass * array([[1 ** 2 + 0.5 ** 2, 0, 0], [0, 11 ** 2 + 0.5 ** 2, 0], [0, 0, 11 ** 2 + 1 ** 2]])) / 12

body.set_angle([0, 0, -pi / 2, 0.3522279392980471, -0.6383816108049025], 'Left')
body.set_angle([0, 0, -pi / 2, 0.3522279392980471, -0.6383816108049025], 'Right')
speed = 4
j = 0

iteration = 0

t = 0
fs = 0
footPosL = [1, 0, 0]
footPosR = [-1, 0, 0]

X = [0, 0, 0]
Y = [0, 0, 0]
Xc = [0, 0, 0]
Yc = [0, 0, 0]

while True:
    ax.axes.set_xlim3d(left=-10, right=10)
    ax.axes.set_ylim3d(bottom=-40, top=40 - speed)
    ax.axes.set_zlim3d(bottom=0, top=40)

    body.get_all_pos()

    body.draw_all(ax)

    ax.scatter(body.find_CoM_Pos()[0][0, 0], body.find_CoM_Pos()[0][0, 1], body.find_CoM_Pos()[0][0, 2])
    ax.grid()

    plt.pause(0.000000000000000001)
    ax.cla()

plt.figure(2)

time = arange(0, t, speed)
plt.plot(time, Y)
plt.plot(time, traj(time)[1])
plt.show()
