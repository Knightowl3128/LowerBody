from numpy import *
from Link import *
from scipy.interpolate import CubicSpline
import rospy
from std_msgs.msg import Float64
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt


class Robot(Link):

    def __init__(self):

        super(Robot, self).__init__(name=None, id=None, mass=None, direction=None, mother=None, child=None)
        self.links_l = []
        self.links_r = []
        # self.CoM = array([[0.0, 0.0, 23.0]])
        # self.CoM = array([[0.0, 0.0, 0]])
        self.CoM = array([[0.0, 0.0, 0.63991906]])  # uthai height
        self.name = 'Original'
        self.time_step = .1
        self.L = 0
        self.P = 0
        self.dL = 0
        self.dP = 0
        # self.ZMP

    def add_link(self, link):

        if link.direction == 'Left':
            self.links_l.append(link)
        if link.direction == 'Right':
            self.links_r.append(link)

    def set_angle(self, angles, direction):
        if direction == 'Left':
            listnow = self.links_l
        elif direction == 'Right':
            listnow = self.links_r
        # for i, j in enumerate(listnow):
        #     if j.name == 'Ankle':
        #         j.dh[0, 0] = -listnow[3].q -listnow[4].q
        #         self.update_angular_dynamics(j.dh[0,0], j)
        #         break
        #     j.dh[0, 0] = angles[i]
        #     self.update_angular_dynamics(j.dh[0, 0], j)
        for i, j in enumerate(listnow):
            if j.name == 'Ankle_p':
                j.dh[0, 0] = -listnow[3].q - listnow[4].q
                self.update_angular_dynamics(j.dh[0, 0], j)
                continue
            if j.name == 'Ankle_r':
                j.dh[0, 0] = pi / 2 + listnow[2].q
                self.update_angular_dynamics(j.dh[0, 0], j)
                continue
            j.dh[0, 0] = angles[i]
            self.update_angular_dynamics(j.dh[0, 0], j)
        # self.get_all_pos()

    def update_angular_dynamics(self, angle, link):
        t = self.time_step
        w = (angle - link.q) / t
        alpha = (w - link.dq) / t
        link.q = angle
        link.dq = w
        link.ddq = alpha

    def update_trans_dynamics(self, link, start, end):
        t = self.time_step
        com_pos = (start + end) / 2
        com_vel = (com_pos - link.com_pos) / t
        com_acc = (com_vel - link.com_vel) / t

        link.com_pos = com_pos
        link.com_vel = com_vel
        link.com_acc = com_acc
        link.start = start
        link.end = end

    def joint_transform_matrix(self, dh):
        theta = dh[0, 0]
        d = dh[0, 1]
        a = dh[0, 2]
        alpha = dh[0, 3]

        trans = mat([[cos(theta), -sin(theta) * cos(alpha), sin(theta) * sin(alpha), a * cos(theta)],
                     [sin(theta), cos(theta) * cos(alpha), -cos(theta) * sin(alpha), a * sin(theta)],
                     [0, sin(alpha), cos(alpha), d],
                     [0, 0, 0, 1]])

        return trans

    def get_all_pos(self):

        listnow = self.links_l

        for i in range(2):
            for index, val in enumerate(listnow):
                if index == 0:
                    com = concatenate([self.CoM, [[1]]], axis=1)

                    rot = array([[1.0, 0, 0],
                                 [0.0, 1, 0],
                                 [0.0, 0, 1],
                                 [0.0, 0, 0]])
                    a = concatenate([rot, com.T], axis=1)
                    m = self.joint_transform_matrix(val.dh)
                    val.trans_mat = a * m
                    self.update_trans_dynamics(val, self.CoM, (val.trans_mat[0:3, 3]).T)
                else:
                    # a = concatenate([rot, val.start], axis=1)
                    m = self.joint_transform_matrix(val.dh)
                    a = listnow[index - 1].trans_mat

                    val.trans_mat = a * m

                    self.update_trans_dynamics(val, listnow[index - 1].end, val.trans_mat[0:3, 3].T)
            listnow = self.links_r
        self.find_momentums()

    def ik_nr(self, pos, direction):
        pass

    def inverse_kinematics(self, pos, direction):

        originPosition = list(self.CoM[0, :])
        if self.name == 'Original':
            off = 1
            a1 = 1  # ankle
            a2 = 11  # femur
            a3 = 9  # thigh
            a4 = 3  # hip
        else:
            off = 0.09
            a1 = 0.045  # ankle
            a2 = .313  # femur
            a3 = .25  # thigh
            a4 = 0.06  # hip
        if direction == 'Left':
            originPosition[0] = originPosition[0] + off
        elif direction == 'Right':
            originPosition[0] = originPosition[0] - off

        originPosition[2] = originPosition[2] - a4
        pos[2] = pos[2] + a1
        for i, val in enumerate(pos):
            pos[i] = pos[i] - originPosition[i]

        X = 0.0000
        Y = 0.0000
        Z = 0.00000

        x2 = pos[0]
        y2 = pos[1]
        z2 = pos[2]

        l = sqrt((x2 - X) ** 2 + (y2 - Y) ** 2 + (z2 - Z) ** 2)
        lz = sqrt((x2 - X) ** 2 + (z2 - Z) ** 2)
        theta1 = math.atan2(x2, z2)
        theta21 = math.atan2(y2, lz)
        c22 = (a2 ** 2 + l ** 2 - a3 ** 2) / (2 * a2 * l)

        s22 = sqrt(1 - c22 ** 2)
        theta22 = math.atan2(s22, c22)
        s23 = (a2 * s22) / a3
        c23 = sqrt(1 - s23 ** 2)

        thetaz = math.atan2(s23, c23)

        ##    return [theta1,theta2,theta3]

        # theta1 is the angle along xz plane
        # theta2 is the thigh angle
        theta2 = theta21 - thetaz
        theta3 = thetaz + theta22
        theta1 = -theta1 + pi / 2
        ##    theta1 = -pi-theta1
        if theta1 > 0:
            theta1 = theta1 - 2 * pi

        return [0, 0, theta1, -theta2, -theta3]

    def find_CoM_Pos(self):
        listnow = self.links_l
        total_m = 0
        com_sum = 0
        for i in range(2):
            for index, val in enumerate(listnow):
                total_m += val.mass
                com_sum += val.mass * val.com_pos
            listnow = self.links_r
        pos = com_sum / total_m

        return pos, total_m

    def find_momentums(self):
        listnow = self.links_l
        H = 0
        P = 0
        L = 0
        for i in range(2):
            w = zeros((3, 1))  # angular velocity in 3x1 matrix
            alpha = zeros((3, 1))  # angular acceleration in 3x1 matrix
            for index, val in enumerate(listnow):
                P += val.mass * val.com_vel
                w = w + listnow[index - 1].trans_mat[0:3, 0:3] * (val.dq * val.a.T)
                R = listnow[index].trans_mat[0:3, 0:3]
                L = L + cross(val.com_pos, (val.mass * val.com_vel)).T + (R * val.I * R.T) * w

        self.dL = (L - self.L) / self.time_step
        self.dP = (P - self.P) / self.time_step
        self.L = L
        self.P = P
        return L, P
        pass

    def find_zmp(self):

        dH = self.dL
        dP = self.dP
        pos, total = self.find_CoM_Pos()
        # print('THis is total mass', end="")
        # print(total)
        # dH = array([[0], [0], [0]])
        # dP = array([[0, 0, 0]])
        x_zmp = (total * 9.81 * pos[0, 0] - dH[1, 0]) / (total * 9.81 + dP[0, 2])
        y_zmp = (total * 9.81 * pos[0, 1] + dH[0, 0]) / (total * 9.81 + dP[0, 2])  # TODO fix this dH its wrong
        return x_zmp, y_zmp

    # def find_zmp(self):
    #     listnow = self.links_l
    #     dH = 0
    #     dP = 0
    #     for i in range(2):
    #         w = zeros((3, 1))  # angular velocity in 3x1 matrix
    #         alpha = zeros((3, 1))  # angular acceleration in 3x1 matrix
    #         for index, val in enumerate(listnow):
    #
    #             dP = dP + val.mass*val.com_acc
    #             R = listnow[index].trans_mat[0:3, 0:3]
    #             w = w +  listnow[index-1].trans_mat[0:3, 0:3]*(val.dq * val.a.T)
    #             alpha = alpha +  listnow[index-1].trans_mat[0:3, 0:3]*(val.ddq * val.a.T)
    #             I = R * val.I * R.T
    #             foo = cross(val.com_pos, val.mass * val.com_acc).T + I * alpha
    #             foo = foo +  cross(w.T, (I * w).T).T
    #             dH = dH+  foo
    #         # print('THis is dP',end="")
    #         # print(dH)
    #         pos, total = self.find_CoM_Pos()
    #         # print('THis is total mass', end="")
    #         # print(total)
    #         dH = array([[0],[0],[0]])
    #         dP = array([[0,0,0]])
    #         x_zmp = (total * 9.81 * pos[0, 0] - dH[1, 0]) / (total * 9.81 + dP[0, 2])
    #         y_zmp = (total * 9.81 * pos[0, 1] - dH[0, 0]) / (total * 9.81 + dP[0, 2])
    #         return x_zmp, y_zmp
    #
    #     pass

    def moveit(self, pos, direction):
        # does not support yaw
        # we assume 1 while loop takes 1 millisecond
        # servo_speed = 230/(pi/3) #ms for 1 rad
        if direction == 'Left':
            listnow = self.links_l
            pos_now = listnow[5].end
        else:
            listnow = self.links_r
            pos_now = listnow[5].end

        initial_angles = []
        for i in listnow:
            initial_angles.append(i.q)
        final_angles = self.inverse_kinematics(pos, direction)
        initial_angles = array([initial_angles[2:5]])

        mid_pos = pos_now

        mid_pos[0, 0] = pos_now[0, 0]
        mid_pos[0, 2] = 5
        mid_pos = list(mid_pos.flat)
        mid_angles = self.inverse_kinematics(mid_pos, direction)

        # time_taken = amax(absolute(angular_dist))*servo_speed
        time_taken = 2
        spline_1 = CubicSpline([0, time_taken / 2, time_taken], [initial_angles[0, 0], mid_angles[2], final_angles[2]],
                               bc_type=(((1, 0)), (1, 0)))
        spline_2 = CubicSpline([0, time_taken / 2, time_taken], [initial_angles[0, 1], mid_angles[3], final_angles[3]],
                               bc_type=(((1, 0)), (1, 0)))
        spline_3 = CubicSpline([0, time_taken / 2, time_taken], [initial_angles[0, 2], mid_angles[4], final_angles[4]],
                               bc_type=(((1, 0)), (1, 0)))

        return spline_1, spline_2, spline_3

    def transition_angle(self, initial_angles, final_angles, time_taken):

        # time_taken = amax(absolute(angular_dist))*servo_speed
        spline_1 = CubicSpline([0, time_taken], [initial_angles[0], final_angles[0]],
                               bc_type=(((1, 0)), (1, 0)))
        spline_2 = CubicSpline([0, time_taken], [initial_angles[1], final_angles[1]],
                               bc_type=(((1, 0)), (1, 0)))
        spline_3 = CubicSpline([0, time_taken], [initial_angles[2], final_angles[2]],
                               bc_type=(((1, 0)), (1, 0)))

        return spline_1, spline_2, spline_3

    def draw_all(self, ax):
        ax.axes.set_xlim3d(left=-0.35, right=0.35)
        ax.axes.set_ylim3d(bottom=-1, top=1)
        ax.axes.set_zlim3d(bottom=0, top=.8)
        listnow = self.links_l
        for i in range(2):
            for index, val in enumerate(listnow):
                if index == 0:

                    ax.scatter(val.start[0, 0], val.start[0, 1], val.start[0, 2], c='red', marker='o', s=50)
                    ax.scatter(val.end[0, 0], val.end[0, 1], val.end[0, 2], c='black', marker='o')

                    ax.plot([val.start[0, 0], val.end[0, 0]], [val.start[0, 1], val.end[0, 1]],
                            [val.start[0, 2], val.end[0, 2]],
                            'black', lw=1)
                else:
                    ax.scatter(val.end[0, 0], val.end[0, 1], val.end[0, 2], c='black', marker='o')

                    ax.plot([val.start[0, 0], val.end[0, 0]], [val.start[0, 1], val.end[0, 1]],
                            [val.start[0, 2], val.end[0, 2]],
                            'black', lw=1)
                    pass

            listnow = self.links_r

    def ros_publish(self):
        msg = Float64()
        listnow = self.links_l
        for i in range(2):
            for index, val in enumerate(listnow):
                if val.pub != None:
                    if val.name == 'Hip_2':
                        msg.data = -val.q - pi / 2
                    elif val.name == 'Ankle_r':
                        msg.data = val.q
                    else:
                        msg.data = -val.q
                    val.pub.publish(msg)
            listnow = self.links_r
        pass
