# meters kilograms seconds
# cm grams seconds

import pybullet as p
import time
import math
import random
import numpy as np
import figures

height = 1.25

def gripper():
  w = 0.1
  len = 0.3
  fig = figures.figure("test_joint", new_inertia = 0, reset_file_count = 0)

  fig.link("body", "sphere", w, 0.2, [0, 0, 0], [0, 0, 0])

  fig.link("stand_arm", "box", [w, len, w], 0.2, [0, len * 0.5, 0], [0, 0, 0])
  fig.link("stand_leg", "box", [w, w, 1.25], 0.2, [0, len + w * 0.5, -0.625], [0,0,0])
  fig.link("stand_foot", "box", [2,2,w], 10, [0, len + w * 0.5, -1.25 + w * 0.5], [0,0,0])
  fig.link("right_arm", "box", [len, w, w], 0.2, [len * 0.5, 0, 0], [0, 0, 0])
  fig.link("right_forearm", "box", [len, w, w], 0.2, [len * 0.5, 0, 0], [0, 0, 0])
  fig.link("left_arm", "box", [len, w, w], 0.2, [-1 * len * 0.5, 0, 0], [0, 0, 0])
  fig.link("left_forearm", "box", [len, w, w], 0.2, [-1 * len * 0.5, 0, 0], [0, 0, 0])

  fig.joint("stand_arm_joint", "body", "stand_arm", "continuous", [0,w,0], [0, 0, 0], [0.5, 1.25])
  fig.joint("stand_leg_joint", "stand_arm", "stand_leg", "continuous", [0,0, 0], [0,0,0], [0,1,0])
  fig.joint("stand_foot_joint", "stand_leg", "stand_foot", "continuous", [0,-0.5,0], [0,0,0], [0,1,0])
  fig.joint("right_joint", "body", "right_arm", "continuous", [w, 0, 0], [0, 0, 0], [0, 1, 0])
  fig.joint("right_forearm_joint", "right_arm", "right_forearm", "continuous", [len, 0, 0], [0, 0.75, 0], [0, 1, 0])
  fig.joint("left_joint", "body", "left_arm", "continuous", [-w, 0, 0], [0, 0, 0], [0, 1, 0])
  fig.joint("left_forearm_joint", "left_arm", "left_forearm", "continuous", [-len, 0, 0], [0, -0.75, 0], [0, 1, 0])

  return fig

def pick_up():
  obj = figures.figure("test_obj", new_inertia = 0, reset_file_count = 0)
  obj.link("obj_body", "sphere", 0.2, 0.2, [0, 0, 0.1], [0, 0, 0])  
  return obj

physicsClient = p.connect(p.GUI)
p.configureDebugVisualizer(p.COV_ENABLE_GUI,0)
p.resetDebugVisualizerCamera(cameraDistance=1.2, cameraYaw=0, cameraPitch=-30, cameraTargetPosition=[0,0,0.2])

p.setGravity(0,0,-10)

plane_id = p.createCollisionShape(p.GEOM_PLANE);
my_plane = p.createMultiBody(baseMass = 0, baseCollisionShapeIndex = plane_id, baseVisualShapeIndex = -1)

fig = gripper()
figure1 = fig.create([0, 0, height], [0, 0, 0])

obj = pick_up()
obj1 = obj.create([0,0,0.2], [0,0,0])

t = 0

for i in range (5000):
  theta = 0.4 * math.sin (t * 0.08) + 0.75
  t += 0.3

  p.setJointMotorControl2(figure1, 3, p.POSITION_CONTROL, targetPosition = theta, force = 0.8)
  p.setJointMotorControl2(figure1, 5, p.POSITION_CONTROL, targetPosition = -1 * theta, force = 0.8)
  p.stepSimulation()

  # look into timestep for pybullet
  time.sleep(1./240.)
  # if (i % 10 == 0):
  #   print(p.getBasePositionAndOrientation(figure1, physicsClient)[0][2])

p.disconnect()