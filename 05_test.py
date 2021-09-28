# meters kilograms seconds
# cm grams seconds

# create outer loop with many different trials to see how robust something is. 
  # One gripper and running it again and again. Take out time.sleep(). Have option for without showing the result
  # aim for 100 loops
# try a few different gripper geometries

import pybullet as p
import time
import math
import random
import numpy as np
import figures
from random import random

height = 1.2
continuous = False

def gripper():
  w = 0.1
  arm_length = 0.5
  if (continuous):
    arm_length = 0.9
  length = 0.3
  fig = figures.figure("test_joint", new_inertia = 0, reset_file_count = 0)

  fig.link("body", "sphere", w, 0.2, [0, 0, 0], [0, 0, 0])

  fig.link("stand_arm", "box", [w, arm_length, w], 0.2, [0, -arm_length * 0.5, 0], [0, 0, 0])
  fig.link("stand_leg", "box", [w, w, height], 0.2, [0, w * 0.5, -height/ 2], [0,0,0])
  fig.link("stand_foot", "box", [2,2,w], 10, [0, w * 0.5, -height + w * 0.5], [0,0,0])
  fig.link("right_arm", "box", [length, w, w], 0.2, [length * 0.5, 0, 0], [0, 0, 0])
  fig.link("right_forearm", "box", [length, w, w], 0.2, [length * 0.5, 0, 0], [0, 0, 0])
  fig.link("left_arm", "box", [length, w, w], 0.2, [-1 * length * 0.5, 0, 0], [0, 0, 0])
  fig.link("left_forearm", "box", [length, w, w], 0.2, [-1 * length * 0.5, 0, 0], [0, 0, 0])

  fig.joint("stand_arm_joint", "body", "stand_arm", "fixed", [0,w + arm_length,0], [0, 0, 0], [0.5, 1.25, 0], [0,0,0])
  if (continuous):
    fig.joint("stand_leg_joint", "stand_arm", "stand_leg", "continuous", [0,0,0], [0,0,0], [1,0,0], [0,0,0])
  else:
    fig.joint("stand_leg_joint", "stand_arm", "stand_leg", "prismatic", [0,0,0], [0,0,0], [0,0,1], [0,-height])
    
  fig.joint("stand_foot_joint", "stand_leg", "stand_foot", "fixed", [0,-0.5,0], [0,0,0], [0,1,0], [0,0,0])
  fig.joint("right_joint", "body", "right_arm", "continuous", [w, 0, 0], [0, 0, 0], [0, 1, 0], [0,0,0])
  fig.joint("right_forearm_joint", "right_arm", "right_forearm", "continuous", [length, 0, 0], [0, 0.75, 0], [0, 1, 0], [0,0,0])
  fig.joint("left_joint", "body", "left_arm", "continuous", [-w, 0, 0], [0, 0, 0], [0, 1, 0], [0,0,0])
  fig.joint("left_forearm_joint", "left_arm", "left_forearm", "continuous", [-length, 0, 0], [0, -0.75, 0], [0, 1, 0], [0,0,0])

  return fig

def pick_up():
  obj = figures.figure("test_obj", reset_file_count = 0)
  # obj.link("obj_body", "sphere", 0.2, 0.2, [0, 0, 0.1], [0, 0, 0]) 
  obj.link("obj_body", "box", [0.3,0.3,0.3], 0.2, [0, 0, 0.1], [0, 0, 0])
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

x = 0
y = 0 
z = 0.2

if (continuous):
  rad = random() * 6.283
  dist = random() * 0.2
  x = math.cos(rad) * dist
  y = math.sin(rad) * dist + 0.375
else:
  x = random() * 0.5 - 0.25
  y = random() * 0.4 - 0.2

obj1 = obj.create([x,y,z], [0,0,0])

t = 0
lift_theta = 0
lift_target = -0.5

grip_theta = 0
grip_target = 1.3
picked_up = False

lift_theta_pris = 0
lift_target_pris = 0.5

for i in range (0,2000):
  t += 0.3

  if (0 <= i <= 500):
    if (continuous):
      lift_theta += lift_target / 500
      p.setJointMotorControl2(figure1, 1, p.POSITION_CONTROL, targetPosition = lift_theta, force = 10)
    else:
      lift_theta_pris += lift_target_pris / 500
      p.setJointMotorControl2(figure1, 1, p.POSITION_CONTROL, targetPosition = lift_theta_pris, force = 100)
  elif (501 <= i <= 1000):
      grip_theta += grip_target / 500
      if (continuous):
        p.setJointMotorControl2(figure1, 1, p.POSITION_CONTROL, targetPosition = lift_theta, force = 10)
      else: 
        p.setJointMotorControl2(figure1, 1, p.POSITION_CONTROL, targetPosition = lift_theta_pris, force = 100)
      p.setJointMotorControl2(figure1, 3, p.POSITION_CONTROL, targetPosition = grip_theta, force = 2)
      p.setJointMotorControl2(figure1, 5, p.POSITION_CONTROL, targetPosition = -1 * grip_theta, force = 0.8)
  elif (lift_theta < 0 or lift_theta_pris > 0):
    if (continuous):
      lift_theta -= lift_target / 500
      p.setJointMotorControl2(figure1, 1, p.POSITION_CONTROL, targetPosition = lift_theta, force = 20)
      p.setJointMotorControl2(figure1, 5, p.POSITION_CONTROL, targetPosition = -1 * grip_theta, force = 0.8)
    else:
      lift_theta_pris -= lift_target_pris / 500
      p.setJointMotorControl2(figure1, 1, p.POSITION_CONTROL, targetPosition = lift_theta_pris, force = 100)
  
  if (p.getBasePositionAndOrientation(obj1)[0][2] > z):
    picked_up = True
  else:
    picked_up = False

  p.stepSimulation()
  time.sleep(1./240.)

print("")
print("***** RESULT *****")

if (picked_up):
  print("SUCCESS: Object successfully picked up!")
else:
  print("FAILED: Object unsuccessfully picked up...")

print("")

p.disconnect()