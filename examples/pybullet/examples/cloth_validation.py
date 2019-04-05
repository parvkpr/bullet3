import pybullet as p
import numpy as np

sim = 4

physicsClient = p.connect(p.GUI)
p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)

# p.setPhysicsEngineParameter(numSolverIterations=10)
# p.setPhysicsEngineParameter(contactBreakingThreshold=0.001)

time_step = 0.005
p.setTimeStep(time_step)

p.setGravity(0, 0, -9.81)
# p.setGravity(0, 0, 0)
plane = p.loadURDF('plane.urdf')

pos1 = np.array([-0.5, 0, 0.5])
pos2 = np.array([0.5, 0, 0.5])
pos3 = np.array([0.5, 0, 1.5])
pos4 = np.array([-0.5, 0, 1.5])

gripper_visual = p.createVisualShape(p.GEOM_SPHERE, radius=0.02, rgbaColor=[1, 0, 0, 1])
gripper_collision = p.createCollisionShape(p.GEOM_SPHERE, radius=0.0001)
# gripper_collision = -1
mass = 1.0 if sim in [3, 4] else 0.0
cloth_attachment1 = p.createMultiBody(baseMass=mass, baseVisualShapeIndex=gripper_visual, baseCollisionShapeIndex=gripper_collision, basePosition=pos1, useMaximalCoordinates=1)
cloth_attachment2 = p.createMultiBody(baseMass=mass, baseVisualShapeIndex=gripper_visual, baseCollisionShapeIndex=gripper_collision, basePosition=pos2, useMaximalCoordinates=1)
cloth_attachment3 = p.createMultiBody(baseMass=mass, baseVisualShapeIndex=gripper_visual, baseCollisionShapeIndex=gripper_collision, basePosition=pos3, useMaximalCoordinates=1)
cloth_attachment4 = p.createMultiBody(baseMass=mass, baseVisualShapeIndex=gripper_visual, baseCollisionShapeIndex=gripper_collision, basePosition=pos4, useMaximalCoordinates=1)

# constraint = p.createConstraint(gripper, 0, cloth_attachment, 0, p.JOINT_POINT2POINT, [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0, 1], [0, 0, 0, 1])
# constraint = p.createConstraint(gripper, 0, cloth_attachment, 0, p.JOINT_FIXED, [0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0, 1], [0, 0, 0, 1])

size = 0.4
block_collision = p.createCollisionShape(p.GEOM_BOX, halfExtents=[size, size, size])
block_visual = p.createVisualShape(p.GEOM_BOX, halfExtents=[size, size, size], rgbaColor=[1, 0, 0, 1])
# block = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=block_collision, baseVisualShapeIndex=block_visual, basePosition=[-1, -0.5, 0.5], useMaximalCoordinates=1)

if sim == 1:
    capsule_pos = np.array([0, -0.5, 1.0])
    capsule_orient = p.getQuaternionFromEuler([np.pi/2.0, 0, 0])
    capsule_collision = p.createCollisionShape(p.GEOM_CAPSULE, radius=0.2, height=0.5)
    capsule_visual = p.createVisualShape(p.GEOM_CAPSULE, radius=0.2, length=0.5, rgbaColor=[1, 0, 0, 1])
    capsule = p.createMultiBody(baseMass=1, baseCollisionShapeIndex=capsule_collision, baseVisualShapeIndex=capsule_visual, basePosition=capsule_pos, baseOrientation=capsule_orient, useMaximalCoordinates=False)

    # capsule_pos = np.array([-0.05, -1, 1.5])
    # capsule_orient = p.getQuaternionFromEuler([0, 0, 0])
    # size = [0.1]*3
    # capsule_collision = p.createCollisionShape(p.GEOM_BOX, halfExtents=size)
    # capsule_visual = p.createVisualShape(p.GEOM_BOX, halfExtents=size, rgbaColor=[1, 0, 0, 1])
    # capsule = p.createMultiBody(baseMass=1, baseCollisionShapeIndex=capsule_collision, baseVisualShapeIndex=capsule_visual, basePosition=capsule_pos, baseOrientation=capsule_orient, useMaximalCoordinates=False)

    # capsule_pos = np.array([-0.05, -1, 1.5])
    # capsule_orient = p.getQuaternionFromEuler([0, 0, 0])
    # capsule_collision = p.createCollisionShape(p.GEOM_SPHERE, radius=0.1)
    # capsule_visual = p.createVisualShape(p.GEOM_SPHERE, radius=0.1, rgbaColor=[1, 0, 0, 1])
    # capsule = p.createMultiBody(baseMass=1, baseCollisionShapeIndex=capsule_collision, baseVisualShapeIndex=capsule_visual, basePosition=capsule_pos, baseOrientation=capsule_orient, useMaximalCoordinates=False)

# bunnyId = p.loadSoftBody('bunny.obj', mass=1, scale=1, collisionMargin=0.02)
if sim in [0, 1, 3, 4]:
    cloth = p.loadClothPatch(31, 31, [-0.5, 0, 0.5], [0.5, 0, 0.5], [-0.5, 0, 1.5], [0.5, 0, 1.5], mass=1, scale=1, orientation=p.getQuaternionFromEuler([0, 0, 0]), bodyAnchorIds=[cloth_attachment1, cloth_attachment2, cloth_attachment3, cloth_attachment4], anchors=[0, 30, 960, 930], collisionMargin=0.02)
    p.clothParams(cloth, kLST=0.1, kAST=1.0, kVST=1.0, kDP=0.01, kDF=0.5, kCHR=1.0, kKHR=1.0, kAHR=1.0, piterations=20)
elif sim == 2:
    cloth = p.loadClothPatch(31, 31, [-0.5, 0, 0.5], [0.5, 0, 0.5], [-0.5, 0, 1.5], [0.5, 0, 1.5], mass=1, scale=1, orientation=p.getQuaternionFromEuler([0, np.pi/4, 0]), bodyAnchorIds=[cloth_attachment1, cloth_attachment2, cloth_attachment3, cloth_attachment4], anchors=[0, 30, 960, 930], collisionMargin=0.02)
    p.clothParams(cloth, kLST=0.1, kAST=1.0, kVST=1.0, kDP=0.01, kDF=0.5, kCHR=1.0, kKHR=1.0, kAHR=0.0, piterations=20)
# double m_kLST;  // Material: Linear stiffness coefficient [0,1]
# double m_kAST;  // Material: Area/Angular stiffness coefficient [0,1]
# double m_kVST;  // Material: Volume stiffness coefficient [0,1]
# double m_kVCF;       // Velocities correction factor (Baumgarte)
# double m_kDP;        // Damping coefficient [0,1]
# double m_kDG;        // Drag coefficient [0,+inf]
# double m_kLF;        // Lift coefficient [0,+inf]
# double m_kPR;        // Pressure coefficient [-inf,+inf]
# double m_kVC;        // Volume conversation coefficient [0,+inf]
# double m_kDF;        // Dynamic friction coefficient [0,1]
# double m_kMT;        // Pose matching coefficient [0,1]
# double m_kCHR;       // Rigid contacts hardness [0,1]
# double m_kKHR;       // Kinetic contacts hardness [0,1]
# double m_kSHR;       // Soft contacts hardness [0,1]
# double m_kAHR;       // Anchors hardness [0,1]

p.configureDebugVisualizer(p.COV_ENABLE_MOUSE_PICKING, 0)
# p.resetDebugVisualizerCamera(cameraDistance=0.5, cameraYaw=0, cameraPitch=-45, cameraTargetPosition=[0, 0, 2])

t = 0
while p.isConnected():
    if sim == 0:
        p.resetBasePositionAndOrientation(cloth_attachment4, p.getBasePositionAndOrientation(cloth_attachment4)[0] + np.array([-0.001, 0, 0.001]), [0, 0, 0, 1])
        p.resetBasePositionAndOrientation(cloth_attachment3, p.getBasePositionAndOrientation(cloth_attachment3)[0] + np.array([0.001, 0, 0.001]), [0, 0, 0, 1])
        p.resetBasePositionAndOrientation(cloth_attachment1, p.getBasePositionAndOrientation(cloth_attachment1)[0] + np.array([-0.001, 0, -0.001]), [0, 0, 0, 1])
        p.resetBasePositionAndOrientation(cloth_attachment2, p.getBasePositionAndOrientation(cloth_attachment2)[0] + np.array([0.001, 0, -0.001]), [0, 0, 0, 1])
    elif sim == 1:
        capsule_pos += np.array([0, 0.0005, 0])
        p.resetBasePositionAndOrientation(capsule, capsule_pos, capsule_orient)
        # p.resetBasePositionAndOrientation(cloth_attachment1, p.getBasePositionAndOrientation(cloth_attachment1)[0] + np.array([0, -0.0005, 0]), [0, 0, 0, 1])
        # p.resetBasePositionAndOrientation(cloth_attachment2, p.getBasePositionAndOrientation(cloth_attachment2)[0] + np.array([0, -0.0005, 0]), [0, 0, 0, 1])
        # p.resetBasePositionAndOrientation(cloth_attachment3, p.getBasePositionAndOrientation(cloth_attachment3)[0] + np.array([0, -0.0005, 0]), [0, 0, 0, 1])
        # p.resetBasePositionAndOrientation(cloth_attachment4, p.getBasePositionAndOrientation(cloth_attachment4)[0] + np.array([0, -0.0005, 0]), [0, 0, 0, 1])
    elif sim == 2 and t == 300:
        p.clothParams(cloth, kLST=0.1, kAST=1.0, kVST=1.0, kDP=0.01, kDF=0.5, kCHR=1.0, kKHR=1.0, kAHR=1.0, piterations=20)
    elif sim == 3 and t > 800:
        p.resetBasePositionAndOrientation(cloth_attachment4, p.getBasePositionAndOrientation(cloth_attachment4)[0] + np.array([0, 0, 0.001]), [0, 0, 0, 1])
        p.resetBasePositionAndOrientation(cloth_attachment3, p.getBasePositionAndOrientation(cloth_attachment3)[0] + np.array([0, 0, 0.001]), [0, 0, 0, 1])
    elif sim == 4:
        if t == 300:
            box_pos = np.array([0, 0, 1.5])
            box_orient = p.getQuaternionFromEuler([0, 0, 0])
            size = [1]*3
            box_collision = p.createCollisionShape(p.GEOM_BOX, halfExtents=size)
            box_visual = p.createVisualShape(p.GEOM_BOX, halfExtents=size, rgbaColor=[1, 0, 0, 1])
            box = p.createMultiBody(baseMass=1000, baseCollisionShapeIndex=box_collision, baseVisualShapeIndex=box_visual, basePosition=box_pos, baseOrientation=box_orient, useMaximalCoordinates=False)
        elif t == 700:
            p.removeBody(box)
        elif t > 700:
            p.resetBasePositionAndOrientation(cloth_attachment4, p.getBasePositionAndOrientation(cloth_attachment4)[0] + np.array([0, 0, 0.001]), [0, 0, 0, 1])
            p.resetBasePositionAndOrientation(cloth_attachment3, p.getBasePositionAndOrientation(cloth_attachment3)[0] + np.array([0, 0, 0.001]), [0, 0, 0, 1])


    p.stepSimulation()
    t += 1

