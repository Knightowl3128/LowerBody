import numpy as np


class Link:

    def __init__(self, name, id, mass, direction, mother, child):
        self.id = id
        self.mother = mother  # Mother ID
        self.direction = direction
        self.name = name  # Name of this link
        self.mass = mass  # Mass of this link
        self.trans_mat = None
        self.child = child  # Child ID of this link
        self.dh = np.zeros((1, 4))  # DH parameter of this link
        self.q = 0  # Angle of link joint
        self.dq = 0  # Joint velocity  [rad/s]
        self.ddq = 0  # Joint acceleration [rad/s^2]
        self.com_pos = np.zeros((1, 3))  # Position of the center of gravity [m]
        self.com_vel = np.zeros((1, 3))
        self.com_acc = np.zeros((1, 3))
        self.I = np.zeros((3, 3))  # Inertia tensor of the center of gravity around [kg.m^2]
        self.p = np.zeros((3, 1))  # Position of this link
        self.R = np.zeros((3, 1))  # Orientation of this link
        self.rot = None  # this argument using to convert self.R to axis angle
        self.a = None  # Axis angle of link
        self.pub = None
        self.process_value = None
        self.start = None
        self.end = None
    def callback(self, data):
        self.process_value = data.process_value
