from LIPM import *
from mono_define import *

fig = plt.figure(1)
ax = fig.add_subplot(111, projection='3d')

body.set_angle([0, 0, -1.5666796536017116, 0.3522279392980471, -0.6383816108049025, pi / 2], 'Left')
body.set_angle([0, 0, -1.5666796536017116, 0.3522279392980471, -0.6383816108049025, pi / 2], 'Right')
j = 0

iteration = 0

foot_height = 0.05
step_size = .1

left_l = True
foot_origin_ds = 0.09

t = 0
switch_timer = 0
X = []
Y = []
Xc = []
Yc = []
angles_l = body.inverse_kinematics([foot_origin_ds, 0, 0], 'Left')
angles_r = body.inverse_kinematics([-foot_origin_ds, 0, 0], 'Right')
body.set_angle(angles_l, 'Left')
body.set_angle(angles_r, 'Right')
body.get_all_pos()
foot_last_pos = [0, 0]
initial_height = 0.70
speed = 0.025
initiate_time = 0.5
zc = 0.4
xsolve, vxsolve, ysolve, vysolve, p_mod = LIPM(speed, initiate_time, zc)
body.time_step = speed
k = 0
Vc = []
Xp = []
time_list = []

while True:
    # body.CoM = array([[-traj(t)[2], -traj(t)[1], 0.6]])
    # body.CoM = array([[ysolve[int(t / speed)] - 0.09, -xsolve[int(t / speed)], 0.6]])
    body.CoM = array([[ysolve[iteration] - 0.09, -xsolve[iteration], initial_height]])
    if abs(round(switch_timer, 3)) == 0:
        print('this is switched %f', t)
        print('This is l6', l_6.end)
        print("This is r6", r_6.end)

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
            switch_timer = initiate_time - speed
            foot_last_pos[0] = r_6.end[0, 0]
            foot_last_pos[1] = r_6.end[0, 1]
            angles_r = body.inverse_kinematics([foot_last_pos[0], foot_last_pos[1], 0], 'Right')
            print(angles_r)
            angles_l = body.inverse_kinematics([foot_origin_ds, spline_y_l(speed), spline_h_l(speed)], 'Left')
            # k = speed
        if not left_l:
            spline_h_r = CubicSpline([0, initiate_time / 2, initiate_time], [0, foot_height, 0],
                                     bc_type=(((1, 0)), (1, 0)))
            spline_y_r = CubicSpline([0, initiate_time],
                                     [body.links_r[6].end[0, 1], -step_multi * step_size + body.links_r[6].end[0, 1]],
                                     bc_type=(((1, 0)), (1, 0)))
            swing_leg = 'Right'
            switch_timer = initiate_time - speed
            # k = speed
            foot_last_pos[0] = l_6.end[0, 0]
            foot_last_pos[1] = l_6.end[0, 1]
            angles_l = body.inverse_kinematics([foot_last_pos[0], foot_last_pos[1], 0], 'Left')
            angles_r = body.inverse_kinematics([-foot_origin_ds, spline_y_r(speed), spline_h_r(speed)], 'Right')
            # print(foot_last_pos)
        left_l = not left_l


    elif abs(round(switch_timer, 4)) > 0:
        switch_timer -= speed

        if swing_leg == 'Left':
            k = initiate_time - switch_timer
            # k +=speed
            if round(k, 2) == initiate_time:
                t += speed
                continue
            angles_l = body.inverse_kinematics([foot_origin_ds, spline_y_l(k), spline_h_l(k)], 'Left')
            angles_r = body.inverse_kinematics([foot_last_pos[0], foot_last_pos[1], 0], 'Right')
            # print("this is in main", end=" ")
            # print(rad2deg(angles_l))

        elif swing_leg == 'Right':
            k = initiate_time - switch_timer
            # k+=speed
            if round(k, 3) == initiate_time:
                t += speed
                continue
            angles_r = body.inverse_kinematics([-foot_origin_ds, spline_y_r(k), spline_h_r(k)], 'Right')
            angles_l = body.inverse_kinematics([foot_last_pos[0], foot_last_pos[1], 0], 'Left')
            # print("this is in main after hit", end=" ")
            # print(rad2deg(angles_l))

    body.set_angle(angles_l, 'Left')
    body.set_angle(angles_r, 'Right')
    body.get_all_pos()
    if iteration >= 3:
        x_zmp, y_zmp = body.find_zmp()
        X.append(x_zmp)
        Y.append(y_zmp)
        time_list.append(t)
        Xc.append(body.find_CoM_Pos()[0][0, 0])
        Yc.append(body.find_CoM_Pos()[0][0, 1])
        Vc.append((core_l.com_vel[0, 0] - k) / speed)
        Xp.append(body.CoM[0, 0])
        # k = core_l.com_vel[0,0]
        ax.scatter(x_zmp, y_zmp, 0)
        ax.scatter(body.find_CoM_Pos()[0][0, 0], body.find_CoM_Pos()[0][0, 1], 0, color='red')
        # plt.figure(2)
        # ax1 = plt.subplot(211)
        # ax2 = plt.subplot(212)
        # #
        # ax1.scatter(t, x_zmp, c='green', s=10)
        # ax2.scatter(t, y_zmp, c='red', s=10)
    iteration += 1
    if iteration == 100:
        break

    # print('This is angle l ',angles_l)
    # print('This is angle r ',angles_r)
    # print('This is l6 ',l_6.end)
    # print('This is r6 ', r_6.end)
    body.draw_all(ax)

    ax.scatter(body.find_CoM_Pos()[0][0, 0], body.find_CoM_Pos()[0][0, 1], body.find_CoM_Pos()[0][0, 2])
    ax.grid()
    t += speed
    plt.pause(0.000000000000000001)
    ax.cla()

plt.figure(2)
# time = linspace(0, 20, len(X))
# plt.plot(time, X)
print(time_list)
plt.plot(time_list, X)
plt.plot(time_list, Xc)
plt.plot(time_list, Xp)

plt.show()
