import pybullet as p
import numpy as np

physicsClient = p.connect(p.GUI)
p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)

# p.setPhysicsEngineParameter(numSolverIterations=10)
# p.setPhysicsEngineParameter(contactBreakingThreshold=0.001)

time_step = 0.005
p.setTimeStep(time_step)

# p.setGravity(0, 0, -9.81)
p.setGravity(0, 0, 0)
plane = p.loadURDF("plane.urdf")
# bunnyId = p.loadSoftBody("bunny.obj")

# cloth_attachment = p.createMultiBody(baseMass=0.01, baseVisualShapeIndex=gripper_visual, basePosition=[-1, 0, 2], useMaximalCoordinates=1)
cloth_attachment = p.createMultiBody(baseMass=0.0, basePosition=[-1, 0, 2], useMaximalCoordinates=1)

gripper = p.loadURDF('tool_scratch.urdf', basePosition=[-1, 0, 2])
p.enableJointForceTorqueSensor(gripper, 0, enableSensor=True)

# constraint = p.createConstraint(gripper, 0, cloth_attachment, 0, p.JOINT_POINT2POINT, [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0, 1], [0, 0, 0, 1])
# constraint = p.createConstraint(gripper, 0, cloth_attachment, 0, p.JOINT_FIXED, [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0, 1], [0, 0, 0, 1])

size = 0.4
block_collision = p.createCollisionShape(p.GEOM_BOX, halfExtents=[size, size, size])
block_visual = p.createVisualShape(p.GEOM_BOX, halfExtents=[size, size, size], rgbaColor=[1, 0, 0, 1])
block = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=block_collision, baseVisualShapeIndex=block_visual, basePosition=[-1, -0.5, 0.5], useMaximalCoordinates=1)

# Anchor points near grasp location
# 952, 953, 2129, 2243, 2244
# cloth = p.loadCloth('hospitalgown_adaptivereduce.obj', scale=1, mass=1, position=[0, 0, 2], orientation=p.getQuaternionFromEuler([0, 0, 0]), bodyAnchorId=cloth_attachment, anchors=[1569, 1570, 1571], collisionMargin=0.01, kLST=0.1, kAST=0.0, kVST=0.0, physicsClientId=0)
cloth = p.loadCloth('hospitalgown_adaptivereduce.obj', scale=1, mass=1, position=[0, 0, 2], orientation=p.getQuaternionFromEuler([0, 0, 0]), bodyAnchorId=cloth_attachment, anchors=[2129, 2243, 2244, 952, 953], collisionMargin=0.01, kLST=0.1, kAST=0.0, kVST=0.0, physicsClientId=0)
# cloth = p.loadCloth('hospitalgown_adaptivereduce.obj', scale=1, mass=1, position=[0, 0, 2], orientation=p.getQuaternionFromEuler([0, 0, 0]), bodyAnchorId=cloth_attachment, anchors=[18, 19, 199, 952, 953, 954, 1416, 1569, 1570, 1571, 1789, 1791, 1879, 1882, 1922, 2062, 2063, 2129, 2238, 2243, 2244, 2292], collisionMargin=0.01, kLST=0.1, kAST=0.0, kVST=0.0, physicsClientId=0)
# Points near rim of sleeve
# 14 points along sleeve edge: 715, 717, 756, 979, 981, 1276, 1615, 1828, 2057, 2173, 2174, 2191, 2277, 2289
# cloth = p.loadCloth('hospitalgown_adaptivereduce.obj', scale=1, mass=1, position=[0, 0, 2], orientation=p.getQuaternionFromEuler([0, 0, 0]), bodyAnchorId=cloth_attachment, anchors=[40, 41, 500, 501, 502, 554, 555, 715, 716, 717, 756, 979, 980, 981, 1276, 1613, 1614, 1615, 1827, 1828, 1962, 2057, 2106, 2114, 2173, 2174, 2191, 2203, 2277, 2289], collisionMargin=0.01, kLST=0.1, kAST=0.0, kVST=0.0, physicsClientId=0)
# cloth = p.loadCloth('hospitalgown_adaptivereduce.obj', scale=1, mass=1, position=[0, 0, 2], orientation=p.getQuaternionFromEuler([0, 0, 0]), bodyAnchorId=cloth_attachment, anchors=list(range(1580, 1600)), collisionMargin=0.01, kLST=0.1, kAST=0.0, kVST=0.0, physicsClientId=0)
p.clothParams(cloth, kDP=0.01, kDF=0.5, kAHR=1, piterations=20)

# joint_names = []
# for j in list(range(p.getNumJoints(cloth))) + [-1]:
#     print(p.getJointInfo(cloth, j))
#     joint_names.append((j, p.getJointInfo(cloth, j)[1]))
# print(joint_names)

# print(p.getJointInfo(cloth, 0))
# print(p.getLinkState(cloth, 0))
# print(p.getDynamicsInfo(cloth, -1))

p.configureDebugVisualizer(p.COV_ENABLE_MOUSE_PICKING, 0)
p.resetDebugVisualizerCamera(cameraDistance=0.5, cameraYaw=0, cameraPitch=-45, cameraTargetPosition=[0, 0, 2])

mouse_down = False
mouse_x, mouse_y = None, None
while p.isConnected():
    p.resetBasePositionAndOrientation(gripper, p.getBasePositionAndOrientation(gripper)[0] + np.array([0, -0.001, 0]), [0, 0, 0, 1])
    # print(p.getJointState(gripper, 0)[2][:3])

    keys = p.getKeyboardEvents()
    mouse = p.getMouseEvents()
    cp = p.getDebugVisualizerCamera()
    if len(mouse) > 0 and mouse[0][0] == 2:
        if mouse[0][-1] == 3:
            mouse_down = True
        elif mouse[0][-1] == 4:
            mouse_down = False
        print(mouse[0], mouse_down)
    if p.B3G_SHIFT in keys and keys[p.B3G_SHIFT] == p.KEY_IS_DOWN and len(mouse) > 0 and mouse[0][0] == 1 and mouse_down:
        print('Moving!')
        p.resetDebugVisualizerCamera(cameraDistance=cp[-2], cameraYaw=cp[-4], cameraPitch=cp[-3], cameraTargetPosition=np.array(cp[-1]) + np.array([(mouse[0][1] - mouse_x)/100.0, (mouse[0][2] - mouse_y)/100.0, 0]))
    if len(mouse) > 0:
        mouse_x = mouse[0][1]
        mouse_y = mouse[0][2]


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