from mono_define import *

fig = plt.figure(1)
ax = fig.add_subplot(111, projection='3d')
speed = 0.05
body.set_angle([0, 0, 2 * pi / 3, 0, 0, 0, 0], 'Left')
body.set_angle([0, 0, pi / 2, 0, 0, 0, 0], 'Right')
body.CoM = array([[0.0, 0.0, 0.764]])
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
    body.CoM = array([[-0.09, 0, 0.7]])
    # v = (body.find_CoM_Pos()[0][0, 0] - bh) / speed
    bh = body.find_CoM_Pos()[0]
    # print(l_6.end)
    print('blah', bh)
    body.set_angle(body.inverse_kinematics([0.09, 0, 0], 'Left'), 'Left')
    body.set_angle(body.inverse_kinematics([-0.09, 0, 0], 'Right'), 'Right')

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

    iteration += 1
    plt.pause(0.000000000000000001)
    ax.cla()
    if iteration == 3:
        break
# plt.figure(2)
#
# time = linspace(0, 0.1 * 51, 51)
# # plt.plot(time, X)
# plt.plot(time, Vc)
# plt.show()
