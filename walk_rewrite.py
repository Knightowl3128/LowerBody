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

left_l = False
foot_origin_ds = 0.09
initiate_time = 2
speed = 0.1
t = 0
switch_timer = 0
X = [0, 0, 0]
Y = [0, 0, 0]
Xc = [0, 0, 0]
Yc = [0, 0, 0]
angles_l = body.inverse_kinematics([foot_origin_ds, 0, 0], 'Left')
angles_r = body.inverse_kinematics([-foot_origin_ds, 0, 0], 'Right')
body.set_angle(angles_l, 'Left')
body.set_angle(angles_r, 'Right')
body.get_all_pos()
foot_last_pos = [0, 0]
body.time_step = speed
xsolve, vxsolve, ysolve, vysolve, p_mod = LIPM(speed, initiate_time)
print(ysolve)
k = 0
Vc = []
