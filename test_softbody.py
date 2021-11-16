# see how many finite elements it uses to make a box
# add rectangular soft body to the gripper and test
# cma parameters:
  # angle and positioning of gripper 
# rigid bodies:
  # figure out a configuration where the units actually work going in and out

import pybullet as p
from time import sleep
import pybullet_data

physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

p.resetSimulation(p.RESET_USE_DEFORMABLE_WORLD)

p.setGravity(0, 0, -10)

planeOrn = [0,0,0,1]#p.getQuaternionFromEuler([0.3,0,0])
planeId = p.loadURDF("plane.urdf", [0,0,0],planeOrn)

boxId = p.loadURDF("cube.urdf", [1.5,0,0.5], useMaximalCoordinates = True)

# p.setAdditionalSearchPath("../../../../bullet3/data")

"""
- Mu: how well it stays together. Bigger = stay more together
- Lambda: (?)
- Damping: hardness. Bigger = more solid
"""
cubeId = p.loadSoftBody("cube.vtk", basePosition = [0.5,0,0.5], scale = 3, mass = 0.05, 
                        useNeoHookean = 1, NeoHookeanMu = 1000, NeoHookeanLambda = 100, 
                        NeoHookeanDamping = 0.1, useSelfCollision = 1, frictionCoeff = .5, 
                        collisionMargin = 0.001)

# cube2Id = p.loadSoftBody("cube.vtk", basePosition = [0.5,-0.5,0.25], scale = 0.5, mass = 1, 
#                         useNeoHookean = 1, NeoHookeanMu = 1000, NeoHookeanLambda = 100, 
#                         NeoHookeanDamping = 0.1, useSelfCollision = 1, frictionCoeff = .5, 
#                         collisionMargin = 0.001)

# p.createSoftBodyAnchor(ballId,20,0,0)
# p.changeVisualShape(ballId, -1, flags=p.VISUAL_SHAPE_DOUBLE_SIDED)

# ----- corners -----
p.createSoftBodyAnchor(cubeId ,4,boxId)
p.createSoftBodyAnchor(cubeId ,5,boxId)
p.createSoftBodyAnchor(cubeId ,6,boxId)
p.createSoftBodyAnchor(cubeId ,7,boxId)

# p.createSoftBodyAnchor(cube2Id ,0,boxId)
# p.createSoftBodyAnchor(cube2Id ,1,boxId)
# p.createSoftBodyAnchor(cube2Id ,2,boxId)
# p.createSoftBodyAnchor(cube2Id ,3,boxId)

# i = 8
# while i <= 27:
#   p.createSoftBodyAnchor(cubeId ,i,boxId)
#   # p.createSoftBodyAnchor(cube2Id ,i,boxId)
#   i += 1

# # ----- right -----
# p.createSoftBodyAnchor(cubeId ,28,boxId)
# p.createSoftBodyAnchor(cubeId ,29,boxId)
# p.createSoftBodyAnchor(cubeId ,30,boxId)
# p.createSoftBodyAnchor(cubeId ,31,boxId)
# p.createSoftBodyAnchor(cubeId ,32,boxId)

# # ----- top -----
# p.createSoftBodyAnchor(cubeId ,33,boxId)
# p.createSoftBodyAnchor(cubeId ,34,boxId)
# p.createSoftBodyAnchor(cubeId ,35,boxId)
# p.createSoftBodyAnchor(cubeId ,36,boxId)
# p.createSoftBodyAnchor(cubeId ,37,boxId)

# # ----- left -----
# p.createSoftBodyAnchor(cubeId ,38,boxId)
# p.createSoftBodyAnchor(cubeId ,39,boxId)
# p.createSoftBodyAnchor(cubeId ,40,boxId)
# p.createSoftBodyAnchor(cubeId ,41,boxId)
# p.createSoftBodyAnchor(cubeId ,42,boxId)

# # ----- left -----
# p.createSoftBodyAnchor(cubeId ,43,boxId)
# p.createSoftBodyAnchor(cubeId ,44,boxId)
# p.createSoftBodyAnchor(cubeId ,45,boxId)
# p.createSoftBodyAnchor(cubeId ,46,boxId)
# p.createSoftBodyAnchor(cubeId ,47,boxId)

p.setTimeStep(0.001)
# p.setPhysicsEngineParameter(sparseSdfVoxelSize=0.25)


#logId = p.startStateLogging(p.STATE_LOGGING_PROFILE_TIMINGS, "perf.json")

while p.isConnected():

  p.stepSimulation()  
  #there can be some artifacts in the visualizer window, 
  #due to reading of deformable vertices in the renderer,
  #while the simulators updates the same vertices
  #it can be avoided using
  #p.configureDebugVisualizer(p.COV_ENABLE_SINGLE_STEP_RENDERING,1)
  #but then things go slower
  p.setGravity(0,0,-10)
  #sleep(1./240.)
  
#p.resetSimulation()
#p.stopStateLogging(logId)