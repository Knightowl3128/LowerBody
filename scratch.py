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
boxId = p.createVisualShape(p.GEOM_SPHERE, 0.1, rgbaColor=[255, 0, 0, 1])
t = 0

mode = p.POSITION_CONTROL

while True:
    id = p.createMultiBody(baseVisualShapeIndex=boxId, basePosition=[10 * sin(t / 100), 5 * sin(t / 10000), 0])
    p.stepSimulation()
    time.sleep(1. / 240.)
    t += 0.1
    p.removeBody(id)
