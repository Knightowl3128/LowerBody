import pybullet as p
import pybullet_data

from Walking import *
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
boxId = p.loadURDF(r"D:\Robotics\Biped\bullet_urdf\bullet.urdf", cubeStartPos, cubeStartOrientation, useFixedBase=0)

t = 0
speed = 0.01
initiate_time = 3
joint_indices = [x for x in range(0, 12)]

mode = p.POSITION_CONTROL

initial_height = 0.70

body.CoM = array([[- 0.09, 0, initial_height]])

spline_1, spline_2, spline_3 = body.transition_angle([pi / 2, 0, 0],
                                                     body.inverse_kinematics([0.0, 0, 0], "Left")[2:],
                                                     initiate_time)
angles_l = [0, 0, pi / 2, 0, 0, 0, 0]
angles_r = [0, 0, pi / 2, 0, 0, 0, 0]
body.set_angle(angles_l, 'Left')
body.set_angle(angles_r, 'Right')
body.get_all_pos()

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

# body.CoM = array([[0.015 - 0.09, 0, initial_height]])

initiate_time = 0.46
T_ssp = 0.6
speed = 1 / 240
T_dsp = 0.08
zc = 0.47
size = 400
body.time_step = speed
step_count = 1
final_angles = array([])

# zmp_id = p.createVisualShape(p.GEOM_SPHERE,0.03,rgbaColor=[255,0,0,1])

com_pos_x = sio.loadmat(r'D:\Matlab Projects\MPC\New MPC\com_x.mat', struct_as_record=False)['ympc'][:, 1]
com_pos_y = sio.loadmat(r'D:\Matlab Projects\MPC\New MPC\com_y.mat', struct_as_record=False)['ympc'][:, 1]
# with open('Online Controller/com_x.npy', 'rb') as f:
#     foo = np.load(f)
#     com_pos_x = np.reshape(foo, (np.size(foo),))
# with open('Online Controller/com_y.npy', 'rb') as f:
#     foo = np.load(f)
#     com_pos_y = np.reshape(foo,(np.size(foo),))

zmp_list = []

zmp_id = p.createVisualShape(p.GEOM_SPHERE, 0.03, rgbaColor=[255, 0, 0, 1])
speed = 1 / 240
mpc_step = 0.005
foo = Walking(1, T_ssp, T_dsp, speed)

while True:
    index = int(round(t / mpc_step, 4))
    body.CoM = array([[(-com_pos_y[index] - 0.09), -com_pos_x[index], initial_height]])
    # body.CoM = array([[ (-com_pos_y[int(t/mpc_step)]-0.09), -com_pos_x[int(t/mpc_step)], initial_height]])
    foot_pos_l, foot_pos_r = foo.sim(t)
    foot_pos_l = [foot_pos_l[1] - 0.09, -foot_pos_l[0], foot_pos_l[2]]
    foot_pos_r = [foot_pos_r[1] - 0.09, -foot_pos_r[0], foot_pos_r[2]]

    body.set_angle(body.inverse_kinematics(foot_pos_l, 'Left'), 'Left')
    body.set_angle(body.inverse_kinematics(foot_pos_r, 'Right'), 'Right')

    body.get_all_pos()
    angles = body.get_current_angles()
    p.setJointMotorControlArray(boxId, joint_indices, mode, angles, forces=max_forces)
    p.stepSimulation()
    # x_zmp, y_zmp = body.find_zmp()
    # id = p.createMultiBody(baseVisualShapeIndex=zmp_id, basePosition=[-y_zmp, x_zmp, 0])
    # p.removeBody(id)
    # time.sleep(1. / 24)
    iteration += 1
    t += speed
p.disconnect()
