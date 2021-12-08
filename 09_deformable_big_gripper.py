# debugging (fixed!):
  # check the mass of the whole soft body or just a part of it
    # keep one of the pads soft and the other one a rigid body with the same mass
    # try an intermediate sized object and see what happens
    # check the density and update the mass based on the mass and density of the smaller one
  # asymmetry 
    # double check the joint indices
    # check constraint points
    # switch things 
# other grippers if the big one doesn't work:
  # 2+ small pads that are offset a little to make a big pad
  # orient the pads differently or in different positions/locations
  # soft pad on one side and rigid pad on the other side
# share code with Turk

import math
import time
import random
import numpy as np
import figures
import random
import cma
import pybullet as p
import pybullet_data
import gmsh
from math import pi

height = 1.2
continuous = False

def init_soft_body(L, B, H):
  gmsh.initialize()

  gmsh.model.add("DFG 3D")

  channel = gmsh.model.occ.addBox(0, 0, 0, L, B, H)

  gmsh.model.occ.synchronize()

  gmsh.model.addPhysicalGroup(3, [1], 1)
  gmsh.model.setPhysicalName(3, 1, "The volume")

  gmsh.model.mesh.generate(3)
  gmsh.write("pad_big.vtk")

  gmsh.fltk.run()
  gmsh.finalize()

def soft_body(boxId):
    """
    - Mu: how well it stays together. Bigger = stay more together
    - Lambda: (?)
    - Damping: hardness. Bigger = more solid
    """
    # baseOrientation = [ 0, -0.3662725, 0, 0.9305076 ]
    cube2Id = p.loadSoftBody("pad_big.vtk", basePosition = [0.15+0.40, -0.15, 1.13-0.235], baseOrientation = [ 0, -0.3989761, 0, 0.9169613 ], 
                            scale = 1, mass = 0.1, 
                            useNeoHookean = 1, NeoHookeanMu = 600, NeoHookeanLambda = 100, 
                            NeoHookeanDamping = 0.1, useSelfCollision = 1, frictionCoeff = .5, 
                            collisionMargin = 0.001)
    
    cubeId = p.loadSoftBody("pad_big.vtk", basePosition = [-0.15-0.42, -0.15, 1.13-0.20], baseOrientation = [ 0, 0.3989761, 0, 0.9169613 ], 
                            scale = 1, mass = 0.1, 
                            useNeoHookean = 1, NeoHookeanMu = 600, NeoHookeanLambda = 100, 
                            NeoHookeanDamping = 0.1, useSelfCollision = 1, frictionCoeff = .5, 
                            collisionMargin = 0.001)

    # ----- corners -----
    p.createSoftBodyAnchor(cube2Id ,4,boxId,3) # 33-40
    p.createSoftBodyAnchor(cube2Id ,5,boxId,3) # 41-44
    p.createSoftBodyAnchor(cube2Id ,6,boxId,3) # 45-52
    p.createSoftBodyAnchor(cube2Id ,7,boxId,3) # 53-56

    p.createSoftBodyAnchor(cubeId ,0,boxId,5)
    p.createSoftBodyAnchor(cubeId ,1,boxId,5)
    p.createSoftBodyAnchor(cubeId ,2,boxId,5)
    p.createSoftBodyAnchor(cubeId ,3,boxId,5)

    i = 8
    while i <= 35:
      p.createSoftBodyAnchor(cube2Id ,i+28,boxId,3)
      p.createSoftBodyAnchor(cubeId ,i,boxId,5)
      i += 1

class gripper():
  def __init__(self, arm_angle, arm_effort, damping, friction):
    self.arm_angle = arm_angle
    self.arm_effort = arm_effort
    self.damping = damping
    self.friction = friction
      
  def make(self):
    w = 0.1
    arm_length = 0.5
    if (continuous):
      arm_length = 0.9
    length = 0.3
    fig = figures.figure("test_joint", new_inertia = 0, reset_file_count = 1)

    fig.link("body", "sphere", w, 0.2, [0, 0, 0], [0, 0, 0])

    fig.link("stand_arm", "box", [w, arm_length, w], 0.2, [0, -arm_length * 0.5, 0], [0, 0, 0])
    fig.link("stand_leg", "box", [w, w, height], 0.2, [0, w * 0.5, -height/ 2], [0,0,0])
    fig.link("stand_foot", "box", [2,2,w], 10, [0, w * 0.5, -height + w * 0.5], [0,0,0])
    fig.link("right_arm", "box", [length, w, w], 0.2, [length * 0.5, 0, 0], [0, 0, 0])
    fig.link("right_forearm", "box", [length, w, w], 0.2, [length * 0.5, 0, 0], [0, 0, 0])
    fig.link("left_arm", "box", [length, w, w], 0.2, [-length * 0.5, 0, 0], [0, 0, 0])
    fig.link("left_forearm", "box", [length, w, w], 0.2, [-1 * length * 0.5, 0, 0], [0, 0, 0])
    # fig.link("left_pad", "box", [0.35, 0.35, 0.01], 0.2, [-0.11,0,0], [0,0,0])
    # fig.link("right_pad", "box", [0.35, 0.35, 0.01], 0.2, [0.11,0,0], [0,0,0])

    # fig.link("left_pad2", "box", [0.3, 0.3, 0.05], 0.2, [-0.11,0,0], [0,0,0])
    # fig.link("right_pad2", "box", [0.3, 0.3, 0.05], 0.05, [0.11,0,0], [0,0,0])

    fig.joint("stand_arm_joint", "body", "stand_arm", "fixed", [0,w + arm_length,0], [0, 0, 0], [0.5, 1.25, 0], [0,0,0])
    if (continuous):
      fig.joint("stand_leg_joint", "stand_arm", "stand_leg", "continuous", [0,0,0], [0,0,0], [1,0,0], [0,0,0])
    else:
      fig.joint("stand_leg_joint", "stand_arm", "stand_leg", "prismatic", [0,0,0], [0,0,0], [0,0,1], [0,-height,0,self.damping,self.friction])
        
    fig.joint("stand_foot_joint", "stand_leg", "stand_foot", "fixed", [0,-0.5,0], [0,0,0], [0,1,0], [0,0,0])

    fig.joint("right_joint", "body", "right_arm", "continuous", [w, 0, 0], [0, 0, 0], [0, 1, 0], [0,0,0])
    fig.joint("right_forearm_joint", "right_arm", "right_forearm", "fixed", [length, 0, 0], [0, self.arm_angle, 0], [0, 1, 0], [0,0,0])

    fig.joint("left_joint", "body", "left_arm", "continuous", [-w, 0, 0], [0, 0, 0], [0, -1, 0], [0,0,0])
    fig.joint("left_forearm_joint", "left_arm", "left_forearm", "fixed", [-length, 0, 0], [0, self.arm_angle * -1, 0], [0, 1, 0], [0,0,0])
    
    # fig.joint("left_pad_joint", "left_forearm", "left_pad", "fixed", [-0.05,0,-0.05], [0,0,0], [0,0,0], [0,0,0])
    # fig.joint("right_pad_joint", "right_forearm", "right_pad", "fixed", [0.05,0,-0.05], [0,0,0], [0,0,0], [0,0,0])

    # fig.joint("left_pad2_joint", "left_forearm", "left_pad2", "fixed", [-0.05,0,-0.09], [0,0,0], [0,0,0], [0,0,0])
    # fig.joint("right_pad2_joint", "right_forearm", "right_pad2", "fixed", [0.05,0,-0.09], [0,0,0], [0,0,0], [0,0,0])

    return fig

def pick_up():
  obj = figures.figure("test_obj", reset_file_count = 0)
  obj.link("obj_body", "sphere", 0.2, 0.2, [0, 0, 0.1], [0, 0, 0]) 
  # obj.link("obj_body", "box", [0.3,0.3,0.3], 0.2, [0, 0, 0.1], [0, 0, 0])
  return obj
    
def cma_optimization(iters, num_params, sigma, cma_opts):
  # init = np.ones(num_params) * 0.5
  es = cma.CMAEvolutionStrategy(num_params * [0], sigma, cma_opts)

  for k in range(iters):
    X = es.ask()
    print("")
    print("iteration = " + str(k))
    print("")

    # es.tell(X, [optimization(x) for x in X]) # list comprehension

    quality_values = [run_simulation(x, iterations = 30) for x in X] # print values here
    print("-----")
    print("Values: " + str(quality_values))
    average = 1 - np.average(quality_values)
    print("Avg: " + str(average))
    print("-----")

    es.tell(X, quality_values)

    if es.stop():
      break
  r = es.result
  return(r[0], r[1])

def run_simulation(params, iterations = 50, gui = False, random_pos = True):
  iterations = iterations
  success = 0
  failure = 0

  arm_angle = params[0]
  arm_effort = 10
  damping = 0.5
  friction = 0.5

  iteration = 0

  random.seed(9)

  while iteration < iterations:
    if (iteration % 10 == 0):
      print(iteration)

    p.resetSimulation(p.RESET_USE_DEFORMABLE_WORLD)
    p.setGravity(0,0,-10)

    p.setAdditionalSearchPath(pybullet_data.getDataPath())

    plane_id = p.createCollisionShape(p.GEOM_PLANE);
    my_plane = p.createMultiBody(baseMass = 0, baseCollisionShapeIndex = plane_id, baseVisualShapeIndex = -1)

    pad_gripper = 0
    pad_gripper = gripper(arm_angle, arm_effort, damping, friction)
    fig = pad_gripper.make()
    figure1 = fig.create([0, 0, height], [0, 0, 0])

    # init_soft_body(0.05, 0.3, 0.3)

    soft_body(figure1)

    # # print the joints and their indices
    # for i in range(7):
    #   joint_info = p.getJointInfo (figure1, i)
    #   print ("joint " + str(i) + " " + str(joint_info[1]))

    obj = pick_up()

    x = 0
    y = 0 
    z = 0.2

    if (continuous):
      rad = random.random() * 6.283
      dist = random.random() * 0.2
      x = math.cos(rad) * dist
      y = math.sin(rad) * dist + 0.375
    else:
      x = random.random() * 0.5 - 0.25
      y = random.random() * 0.4 - 0.2

    if not random_pos:
      x = -0.1366470703094756
      y = 0.18491801433375316

    obj1 = obj.create([x,y,z], [0,0,0])

    t = 0
    lift_theta = 0
    lift_target = -0.5

    grip_theta = 0
    grip_theta_l = 0
    grip_target = 2
    picked_up = False

    lift_theta_pris = 0
    lift_target_pris = 0.5

    right_joint_ind = 3
    left_joint_ind = 5

    p.setTimeStep(0.0001)

    for i in range (0,6000):
      t += 0.3

      if (0 <= i <= 1000):
        if (continuous):
          lift_theta += lift_target / 500
          p.setJointMotorControl2(figure1, 1, p.POSITION_CONTROL, targetPosition = lift_theta, force = 100)
          p.setJointMotorControl2(figure1, right_joint_ind, p.POSITION_CONTROL, targetPosition = grip_theta, force = 200)
          p.setJointMotorControl2(figure1, left_joint_ind, p.POSITION_CONTROL, targetPosition = grip_theta, force = 200)
        else:
          lift_theta_pris += lift_target_pris / 500
          p.setJointMotorControl2(figure1, 1, p.POSITION_CONTROL, targetPosition = lift_theta_pris, force = 100)
          p.setJointMotorControl2(figure1, right_joint_ind, p.POSITION_CONTROL, targetPosition = grip_theta, force = 200)
          p.setJointMotorControl2(figure1, left_joint_ind, p.POSITION_CONTROL, targetPosition = grip_theta, force = 200)
      elif (1001 <= i <= 1500):
          grip_theta += grip_target / 500
          # grip_theta_l -= grip_target / 500
          if (continuous):
            p.setJointMotorControl2(figure1, 1, p.POSITION_CONTROL, targetPosition = lift_theta, force = 100)
            p.setJointMotorControl2(figure1, right_joint_ind, p.POSITION_CONTROL, targetPosition = grip_theta, force = 200)
            p.setJointMotorControl2(figure1, left_joint_ind, p.POSITION_CONTROL, targetPosition = grip_theta, force = 200)
          else: 
            p.setJointMotorControl2(figure1, 1, p.POSITION_CONTROL, targetPosition = lift_theta_pris, force = 100)
            p.setJointMotorControl2(figure1, right_joint_ind, p.POSITION_CONTROL, targetPosition = grip_theta, force = 200)
            p.setJointMotorControl2(figure1, left_joint_ind, p.POSITION_CONTROL, targetPosition = grip_theta, force = 200)
      elif (lift_theta < 0 or lift_theta_pris > 0):
        if (continuous):
          lift_theta -= lift_target / 500
          p.setJointMotorControl2(figure1, 1, p.POSITION_CONTROL, targetPosition = lift_theta, force = 100)
          p.setJointMotorControl2(figure1, right_joint_ind, p.POSITION_CONTROL, targetPosition = grip_theta, force = 200)
          p.setJointMotorControl2(figure1, left_joint_ind, p.POSITION_CONTROL, targetPosition = grip_theta, force = 200)
        else:
          lift_theta_pris -= lift_target_pris / 500
          p.setJointMotorControl2(figure1, 1, p.POSITION_CONTROL, targetPosition = lift_theta_pris, force = 300)
          p.setJointMotorControl2(figure1, right_joint_ind, p.POSITION_CONTROL, targetPosition = grip_theta, force = 200)
          p.setJointMotorControl2(figure1, left_joint_ind, p.POSITION_CONTROL, targetPosition = grip_theta, force = 200)

      p.stepSimulation()
      # if gui:
        # time.sleep(1./10000.)
        # time.sleep(1./240.)

    if (p.getBasePositionAndOrientation(obj1)[0][2] > z + 0.3):
      picked_up = True
      success += 1
      print("Success at: " + str(x) + ", " + str(y))
    else:
      picked_up = False
      failure += 1
      print("Failure at: " + str(x) + ", " + str(y))

    iteration += 1

  success_rate = success/(success + failure)
  return (1 - success_rate)

  # print("-----")
  # print("Success rate: " + str(success_rate))

# physicsClient = p.connect(p.DIRECT)

# cma_opts = {'BoundaryHandler': cma.BoundTransform, 'bounds': [0, 10]}
# result = cma_optimization(20, 3, 0.1, cma_opts)

physicsClient = p.connect(p.GUI)
p.configureDebugVisualizer(p.COV_ENABLE_GUI,0)
p.resetDebugVisualizerCamera(cameraDistance=2, cameraYaw=0, cameraPitch=-30, cameraTargetPosition=[0,0,0.2])

# 90-94% success rate!
params = [0.75]
result = 1 - run_simulation(params, iterations = 50, gui = False, random_pos = True)

print("")
print("----- RESULT -----")
print(result)
print("")

p.disconnect()