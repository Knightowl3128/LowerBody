import pybullet as p
import time
import pybullet_data
from Robot import *
from numpy import *
from LIPM_with_dsupport import *
from mono_define import *

physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())  # optionally
p.setGravity(0, 0, -9.81)
planeId = p.loadURDF("plane.urdf")
cubeStartPos = [0, 0, 0.764]
cubeStartOrientation = p.getQuaternionFromEuler([0, 0, 0])
# for ubuntu
# boxId = p.loadURDF("/home/navaneeth/mysim/src/mono/urdf/mono_bullet.urdf", cubeStartPos, cubeStartOrientation)

# for windows
boxId = p.loadURDF(r"D:\Robotics\Biped\bullet_urdf\bullet.urdf", cubeStartPos, cubeStartOrientation)

t = 0
speed = 0.005
initiate_time = 3
joint_indices = [x for x in range(0, 12)]

mode = p.POSITION_CONTROL

initial_height = 0.70

body.CoM = array([[0.015 - 0.09, 0, initial_height]])

spline_1, spline_2, spline_3 = body.transition_angle([pi / 2, 0, 0],
                                                     body.inverse_kinematics([0.09, 0, 0], "Left")[2:],
                                                     initiate_time)
angles_l = [0, 0, pi / 2, 0, 0, 0, 0]
angles_r = [0, 0, pi / 2, 0, 0, 0, 0]
body.set_angle(angles_l, 'Left')
body.set_angle(angles_r, 'Right')
body.get_all_pos()

print(p.getJointInfo(boxId, 1)[10])
print('-------------------')
print(body.find_CoM_Pos())
max_forces = []
for i in joint_indices:
    # max_forces.append(p.getJointInfo(boxId,i)[10])
    max_forces.append(10)
while t <= initiate_time:
    angles_l = [0, 0, spline_1(t), spline_2(t), spline_3(t), pi / 2]
    angles_r = [0, 0, spline_1(t), spline_2(t), spline_3(t), pi / 2]

    body.set_angle(angles_l, 'Left')
    body.set_angle(angles_r, 'Right')
    body.get_all_pos()
    angles = body.get_current_angles()
    p.setJointMotorControlArray(boxId, joint_indices, mode, angles)
    t += speed
    p.stepSimulation()
    # time.sleep(1. / 240.)

t = 0
foot_height = 0.05
step_size = .1
iteration = 0
switch_timer = 0
left_l = True
foot_origin_ds = 0.09

foot_last_pos = [0, 0]

body.CoM = array([[0.015 - 0.09, 0, initial_height]])

initiate_time = 0.46
T_dbl = 0.08
zc = 0.47
size = 400
xsolve, vxsolve, ysolve, vysolve, p_mod = LIPM(speed, initiate_time, T_dbl, zc, size)
body.time_step = speed
step_count = 1
final_angles = array([])

# zmp_id = p.createVisualShape(p.GEOM_SPHERE,0.03,rgbaColor=[255,0,0,1])


while True:
    if iteration >= len(ysolve) - 5:
        print("End of walk------------", step_count)
        break
    # body.ros_subscribe()
    body.CoM = array([[ysolve[iteration] - 0.09, -xsolve[iteration], initial_height]])
    # send_pelvis_data(body.CoM)
    if abs(round(switch_timer, 3)) == 0:
        print(step_count)
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
            # spline_y_l = CubicSpline([0, initiate_time],
            #                          [body.links_l[6].end[0, 1], -step_multi * step_size + body.links_l[6].end[0, 1]],
            #                          bc_type=(((1, 0)), (1, 0)))
            spline_y_l = CubicSpline([0, initiate_time],
                                     [body.links_l[6].end[0, 1], -p_mod[0, step_count]],
                                     bc_type=(((1, 0)), (1, 0)))
            spline_x_l = CubicSpline([0, initiate_time],
                                     [body.links_l[6].end[0, 0], p_mod[1, step_count]],
                                     bc_type=(((1, 0)), (1, 0)))

            swing_leg = 'Left'
            switch_timer = initiate_time + T_dbl
            ds_timer = T_dbl
            dbl_phase = False
            foot_last_pos[0] = r_6.end[0, 0]
            foot_last_pos[1] = r_6.end[0, 1]
            angles_r = body.inverse_kinematics([foot_last_pos[0], foot_last_pos[1], 0], 'Right')
            angles_l = body.inverse_kinematics([spline_x_l(0), spline_y_l(0), spline_h_l(0)], 'Left')
            step_count += 1
            # k = speed
        if not left_l:
            spline_h_r = CubicSpline([0, initiate_time / 2, initiate_time], [0, foot_height, 0],
                                     bc_type=((1, 0), (1, 0)))
            # spline_y_r = CubicSpline([0, initiate_time],
            #                          [body.links_r[6].end[0, 1], -step_multi * step_size + body.links_r[6].end[0, 1]],
            #                          bc_type=((1, 0), (1, 0)))
            spline_y_r = CubicSpline([0, initiate_time],
                                     [body.links_r[6].end[0, 1], -p_mod[0, step_count]],
                                     bc_type=(((1, 0)), (1, 0)))
            spline_x_r = CubicSpline([0, initiate_time],
                                     [body.links_r[6].end[0, 0], p_mod[1, step_count]],
                                     bc_type=(((1, 0)), (1, 0)))
            swing_leg = 'Right'
            switch_timer = initiate_time + T_dbl
            ds_timer = T_dbl
            dbl_phase = False
            # k = speed
            foot_last_pos[0] = l_6.end[0, 0]
            foot_last_pos[1] = l_6.end[0, 1]
            angles_l = body.inverse_kinematics([foot_last_pos[0], foot_last_pos[1], 0], 'Left')
            angles_r = body.inverse_kinematics([spline_x_r(0), spline_y_r(0), spline_h_r(0)], 'Right')
            step_count += 1
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

            if abs(round(switch_timer, 4)) == 0:
                switch_timer = 0
                t += speed
                continue
            angles_l = body.inverse_kinematics([spline_x_l(k), spline_y_l(k), spline_h_l(k)], 'Left')
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
            angles_r = body.inverse_kinematics([spline_x_r(k), spline_y_r(k), spline_h_r(k)], 'Right')
            angles_l = body.inverse_kinematics([foot_last_pos[0], foot_last_pos[1], 0], 'Left')
            # print("this is in main after hit", end=" ")
            # print(rad2deg(angles_l))
    body.set_angle(angles_l, 'Left')
    body.set_angle(angles_r, 'Right')
    # x_zmp, y_zmp = body.find_zmp()
    # id = p.createMultiBody(baseVisualShapeIndex=zmp_id, basePosition=[-y_zmp, x_zmp, 0])

    body.get_all_pos()
    angles = body.get_current_angles()
    p.setJointMotorControlArray(boxId, joint_indices, mode, angles, forces=max_forces)
    p.stepSimulation()
    # p.removeBody(id)
    # time.sleep(1. / 240.)
    iteration += 1
p.disconnect()
