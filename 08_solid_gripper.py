import pybullet as p
import time
import math
import random
import numpy as np
import figures
import random
import cma

height = 1.2
continuous = False

class gripper():
  def __init__(self, dim_x, dim_y, lower_limit, upper_limit, effort_limit, arm_effort, damping, friction):
    self.dim_x = dim_x
    self.dim_y = dim_y
    self.lower_limit = lower_limit 
    self.upper_limit = upper_limit
    self.effort_limit = effort_limit
    self.arm_effort = arm_effort
    self.num_units = dim_x * dim_y * 2
    self.damping = damping
    self.friction = friction
    if (dim_x > 0 and dim_y > 0):
      self.use_units = True
    else:
      self.use_units = False

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
    fig.link("left_pad", "box", [0.3, 0.3, 0.05], 0.2, [-0.11,0,0], [0,0,0])
    fig.link("right_pad", "box", [0.3, 0.3, 0.05], 0.2, [0.11,0,0], [0,0,0])

    fig.joint("stand_arm_joint", "body", "stand_arm", "fixed", [0,w + arm_length,0], [0, 0, 0], [0.5, 1.25, 0], [0,0,0])
    if (continuous):
      fig.joint("stand_leg_joint", "stand_arm", "stand_leg", "continuous", [0,0,0], [0,0,0], [1,0,0], [0,0,0])
    else:
      fig.joint("stand_leg_joint", "stand_arm", "stand_leg", "prismatic", [0,0,0], [0,0,0], [0,0,1], [0,-height,0,self.damping,self.friction])
        
    fig.joint("stand_foot_joint", "stand_leg", "stand_foot", "fixed", [0,-0.5,0], [0,0,0], [0,1,0], [0,0,0])

    fig.joint("right_joint", "body", "right_arm", "continuous", [w, 0, 0], [0, 0, 0], [0, 1, 0], [0,0,0])
    fig.joint("right_forearm_joint", "right_arm", "right_forearm", "fixed", [length, 0, 0], [0, 0.75, 0], [0, 1, 0], [0,0,0])

    fig.joint("left_joint", "body", "left_arm", "continuous", [-w, 0, 0], [0, 0, 0], [0, -1, 0], [0,0,0])
    fig.joint("left_forearm_joint", "left_arm", "left_forearm", "fixed", [-length, 0, 0], [0, -0.75, 0], [0, 1, 0], [0,0,0])

    fig.joint("left_pad_joint", "left_forearm", "left_pad", "fixed", [-0.05,0,-0.09], [0,0,0], [0,0,0], [0,0,0])
    fig.joint("right_pad_joint", "right_forearm", "right_pad", "fixed", [0.05,0,-0.09], [0,0,0], [0,0,0], [0,0,0])

    if (self.use_units):
      weight = 0.64 / self.num_units
      pad_width = 2 * w
      unit_w_x = pad_width / self.dim_x
      unit_w_y = pad_width / self.dim_y

      i = 0
      j = 0
      while i < self.dim_x:
        j = 0
        while j < self.dim_y:
          name_r = "right_contact_" + str(i) + str(j)
          name_l = "left_contact_" + str(i) + str(j)
          fig.link(name_r, "box", [unit_w_x, unit_w_y, 0.2], weight, [-unit_w_x * 0.5, -unit_w_y * 0.5, -0.1], [0,0,0])
          fig.link(name_l, "box", [unit_w_x, unit_w_y, 0.2], weight, [unit_w_x * 0.5, -unit_w_y * 0.5, -0.1], [0,0,0])
          j += 1
        i += 1

      k = 0
      l = 0
      while k < self.dim_x:
        l = 0
        while l < self.dim_y:
          x_pos_r = length - k * unit_w_x
          x_pos_l = -length + k * unit_w_x
          y_pos = (l - (self.dim_y/2 - 1))* unit_w_y
          name_r = "right_contact_" + str(k) + str(l)
          name_l = "left_contact_" + str(k) + str(l)
          joint_r = "right_joint_" + str(k) + str(l)
          joint_l = "left_joint_" + str(k) + str(l)
          fig.joint(joint_r, "right_forearm", name_r, "prismatic", [x_pos_r, y_pos, -0.05], [0,0,0], [0,0,1], [self.lower_limit,self.upper_limit,self.effort_limit,self.damping,self.friction])
          fig.joint(joint_l, "left_forearm", name_l, "prismatic", [x_pos_l, y_pos, -0.05], [0,0,0], [0,0,1], [self.lower_limit,self.upper_limit,self.effort_limit,self.damping,self.friction])
          l += 1
        k += 1

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

  pad_size = 0

  start = params[0]
  add = params[1]
  lower_limit = start
  upper_limit = start + add

  unit_effort = 1
  arm_effort = params[2]
  damping = 0.5
  friction = 0.5

  iteration = 0

  random.seed(10)

  while iteration < iterations:
    if (iteration % 10 == 0):
      print(iteration)

    p.resetSimulation()
    p.setGravity(0,0,-10)

    plane_id = p.createCollisionShape(p.GEOM_PLANE);
    my_plane = p.createMultiBody(baseMass = 0, baseCollisionShapeIndex = plane_id, baseVisualShapeIndex = -1)

    pad_gripper = 0
    pad_gripper = gripper(pad_size, pad_size, lower_limit, upper_limit, unit_effort, arm_effort, damping, friction)
    fig = pad_gripper.make()
    figure1 = fig.create([0, 0, height], [0, 0, 0])

    right_joint_ind = 3
    left_joint_ind = 6 + pad_gripper.dim_x * pad_gripper.dim_y

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
      x = -0.16681812710337623
      y = 0.12297990080990229

    # if (iteration % 5 == 0):
    #   print("----- pos -----")
    #   print(x)
    #   print(y)

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

    for i in range (0,2000):
      t += 0.3

      if (0 <= i <= 500):
        if (continuous):
          lift_theta += lift_target / 500
          p.setJointMotorControl2(figure1, 1, p.POSITION_CONTROL, targetPosition = lift_theta, force = 100)
        else:
          lift_theta_pris += lift_target_pris / 500
          p.setJointMotorControl2(figure1, 1, p.POSITION_CONTROL, targetPosition = lift_theta_pris, force = 100)
      elif (501 <= i <= 1000):
          grip_theta += grip_target / 500
          grip_theta_l -= grip_target / 500
          if (continuous):
            p.setJointMotorControl2(figure1, 1, p.POSITION_CONTROL, targetPosition = lift_theta, force = 100)
            p.setJointMotorControl2(figure1, right_joint_ind, p.POSITION_CONTROL, targetPosition = grip_theta, force = 100)
            p.setJointMotorControl2(figure1, left_joint_ind, p.POSITION_CONTROL, targetPosition = grip_theta, force = 100)
          else: 
            p.setJointMotorControl2(figure1, 1, p.POSITION_CONTROL, targetPosition = lift_theta_pris, force = 100)
            p.setJointMotorControl2(figure1, right_joint_ind, p.POSITION_CONTROL, targetPosition = grip_theta, force = pad_gripper.arm_effort)
            p.setJointMotorControl2(figure1, left_joint_ind, p.POSITION_CONTROL, targetPosition = grip_theta, force = pad_gripper.arm_effort)

            if (pad_gripper.use_units):
              i = 0
              while i < (pad_gripper.num_units / 2):
                unit_r = right_joint_ind + i + 2
                unit_l = left_joint_ind + i + 2
                p.setJointMotorControl2(figure1, unit_r, p.POSITION_CONTROL, force = pad_gripper.effort_limit)
                p.setJointMotorControl2(figure1, unit_l, p.POSITION_CONTROL, force = pad_gripper.effort_limit)
                i += 1
      elif (lift_theta < 0 or lift_theta_pris > 0):
        if (continuous):
          lift_theta -= lift_target / 500
          p.setJointMotorControl2(figure1, 1, p.POSITION_CONTROL, targetPosition = lift_theta, force = 100)
          p.setJointMotorControl2(figure1, right_joint_ind, p.POSITION_CONTROL, targetPosition = grip_theta, force = 100)
          p.setJointMotorControl2(figure1, left_joint_ind, p.POSITION_CONTROL, targetPosition = grip_theta, force = 100)
        else:
          lift_theta_pris -= lift_target_pris / 500
          p.setJointMotorControl2(figure1, 1, p.POSITION_CONTROL, targetPosition = lift_theta_pris, force = 100)
          p.setJointMotorControl2(figure1, right_joint_ind, p.POSITION_CONTROL, targetPosition = grip_theta, force = pad_gripper.arm_effort)
          p.setJointMotorControl2(figure1, left_joint_ind, p.POSITION_CONTROL, targetPosition = grip_theta, force = pad_gripper.arm_effort)

          if (pad_gripper.use_units):
            i = 0
            while i < (pad_gripper.num_units / 2):
              unit_r = right_joint_ind + i
              unit_l = left_joint_ind + i
              # p.setJointMotorControl2(figure1, unit_r, p.POSITION_CONTROL, force = pad_gripper.effort_limit)
              # p.setJointMotorControl2(figure1, unit_l, p.POSITION_CONTROL, force = pad_gripper.effort_limit)
              i += 1

      p.stepSimulation()
      if gui:
        # time.sleep(1./10000.)
        time.sleep(1./240.)

    if (p.getBasePositionAndOrientation(obj1)[0][2] > z + 0.3):
      picked_up = True
      success += 1
      # print("Success at: " + str(x) + ", " + str(y))
    else:
      picked_up = False
      failure += 1
      # print("Failure at: " + str(x) + ", " + str(y))

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
p.resetDebugVisualizerCamera(cameraDistance=1.2, cameraYaw=0, cameraPitch=-30, cameraTargetPosition=[0,0,0.2])

# start = 0.2488432
# add = 0.0179326 

# start = 0
# add = 0.07
# effort = 0.18237019

start = 5.49867916e-04
add = 1.45333932e-04
effort = 6.36396532e-11

params = [start, add, effort]

# 32%: small pad 
# 74%: big pad
result = 1 - run_simulation(params, iterations = 1, gui = True, random_pos = True)

print("")
print("----- RESULT -----")
print(result)
print("")

p.disconnect()