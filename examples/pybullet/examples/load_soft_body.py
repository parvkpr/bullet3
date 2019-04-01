import pybullet as p
import numpy as np

physicsClient = p.connect(p.GUI)
p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)

# p.setPhysicsEngineParameter(numSolverIterations=10)
# p.setPhysicsEngineParameter(contactBreakingThreshold=0.001)

p.setGravity(0, 0, -9.81)
# p.setGravity(0, 0, 0)
plane = p.loadURDF("plane.urdf")
# bunnyId = p.loadSoftBody("bunny.obj")
# shirt = p.loadSoftBody("pillows_3.obj", 0.001)
# shirt = p.loadSoftBody("Tshirt_obj.obj", 0.01)

# cube = p.loadURDF('cube_small.urdf', [0, 0.035, 1.75], useFixedBase=True, useMaximalCoordinates=1)
# cube2 = p.loadURDF('cube_small.urdf', [0, -0.035, 1.75], useFixedBase=True, useMaximalCoordinates=1)
# cube = p.loadURDF('cube_small.urdf', [0, 0.025, 0], useFixedBase=True, useMaximalCoordinates=1)
# cube2 = p.loadURDF('cube_small.urdf', [0, -0.5, 0], useFixedBase=False, useMaximalCoordinates=1)

size = 0.05
gripper_collision = p.createCollisionShape(p.GEOM_BOX, halfExtents=[size, size, size])
gripper_visual = p.createVisualShape(p.GEOM_BOX, halfExtents=[size, size, size], rgbaColor=[1, 0, 0, 1])
block = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=gripper_collision, baseVisualShapeIndex=gripper_visual, basePosition=[-5, 0, 1], useMaximalCoordinates=1)
block = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=gripper_collision, baseVisualShapeIndex=gripper_visual, basePosition=[-5, 1, 1], useMaximalCoordinates=1)
block = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=gripper_collision, baseVisualShapeIndex=gripper_visual, basePosition=[-5, 2, 1], useMaximalCoordinates=1)
block = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=gripper_collision, baseVisualShapeIndex=gripper_visual, basePosition=[-5, 3, 1], useMaximalCoordinates=1)
# block = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=gripper_collision, baseVisualShapeIndex=gripper_visual, basePosition=[-5, 4, 1], useMaximalCoordinates=1)

# cloth_attachment = p.createMultiBody(baseMass=0.01, baseVisualShapeIndex=gripper_visual, basePosition=[-1, 0, 2], useMaximalCoordinates=1)
cloth_attachment = p.createMultiBody(baseMass=0.01, basePosition=[-1, 0, 2], useMaximalCoordinates=1)

# gripper_collision2 = p.createCollisionShape(p.GEOM_BOX, halfExtents=[size, size, size])
# gripper_visual2 = p.createVisualShape(p.GEOM_BOX, halfExtents=[size, size, size], rgbaColor=[1, 0, 0, 1])
# gripper = p.createMultiBody(baseMass=1, baseCollisionShapeIndex=gripper_collision, baseVisualShapeIndex=gripper_visual, basePosition=[-1, 0, 2.2], linkMasses=[1], linkCollisionShapeIndices=[gripper_collision], linkVisualShapeIndices=[gripper_visual], linkPositions=[[0, 0, -0.2]], linkOrientations=[[0, 0, 0, 1]], linkInertialFramePositions=[[0, 0, 0]], linkInertialFrameOrientations=[[0, 0, 0, 1]], linkParentIndices=[0], linkJointTypes=[p.JOINT_FIXED], linkJointAxis=[[0, 0, 1]])
gripper = p.loadURDF('tool_scratch.urdf', basePosition=[-1, 0, 2])
p.enableJointForceTorqueSensor(gripper, 0, enableSensor=True)

# constraint = p.createConstraint(gripper, 0, cloth_attachment, 0, p.JOINT_POINT2POINT, [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0, 1], [0, 0, 0, 1])
constraint = p.createConstraint(gripper, 0, cloth_attachment, 0, p.JOINT_FIXED, [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0, 1], [0, 0, 0, 1])

size = 0.4
block_collision = p.createCollisionShape(p.GEOM_BOX, halfExtents=[size, size, size])
block_visual = p.createVisualShape(p.GEOM_BOX, halfExtents=[size, size, size], rgbaColor=[1, 0, 0, 1])
block = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=block_collision, baseVisualShapeIndex=block_visual, basePosition=[-1, -0.5, 0.5], useMaximalCoordinates=1)

# cloth = p.loadSoftBody('smoothed_mesh2.obj', scale=1, mass=10, collisionMargin=0.01)
# Loads, fast, less vertices, but sleeves not attached
# cloth = p.loadSoftBody('fullgown1.obj', scale=1, mass=1, collisionMargin=0.01)
# cloth = p.loadCloth('fullgown1.obj', scale=1, mass=1, collisionMargin=0.01)
# Loads, but many vertices and is slow
# cloth = p.loadSoftBody('fullgown_midpoint2.obj', scale=1, mass=1, collisionMargin=0.01)
# p.resetBasePositionAndOrientation(cloth, [0, 0, 100], [0, 0, 0, 1])

cloth = p.loadCloth('hospitalgown_adaptivereduce.obj', scale=1, mass=1, position=[-1, 0, 1.2], orientation=p.getQuaternionFromEuler([0, 0, 0]), bodyAnchorId=cloth_attachment, anchors=[99, 100, 101], collisionMargin=0.01, physicsClientId=0)
p.clothParams(cloth, kDF=0.5, kAHR=1, piterations=20)
print(cloth)

# joint_names = []
# for j in list(range(p.getNumJoints(cloth))) + [-1]:
#     print(p.getJointInfo(cloth, j))
#     joint_names.append((j, p.getJointInfo(cloth, j)[1]))
# print(joint_names)

# print(p.getJointInfo(cloth, 0))
# print(p.getLinkState(cloth, 0))
# print(p.getDynamicsInfo(cloth, -1))

while p.isConnected():
    p.resetBasePositionAndOrientation(gripper, p.getBasePositionAndOrientation(gripper)[0] + np.array([0, -0.001, 0]), [0, 0, 0, 1])
    # print(p.getJointState(gripper, 0)[2][:3])

    p.stepSimulation()

    # print(p.getClosestPoints(cloth, block, distance=10))

    total_force = 0
    force_on_block = 0
    for c in p.getContactPoints():
        print('Force on block!')
        total_force += c[9]
        body_B = c[2]
        if body_B != plane:
            force_on_block += c[9]
    # for c in p.getContactPoints():
    #     total_force += c[9]
    # print('Total force:', total_force, 'Force on block:', force_on_block)

    pass
