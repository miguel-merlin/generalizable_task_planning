import pybullet as p
import pybullet_data
import os
import math
import numpy as np
from utils import current_joint_positions

# Change this variable to change how many simulations are run
num_sims = 10

# Joint states from each simulation is published here
sim_results = []


for loop in range(num_sims):
    runsim = True

    p.connect(p.GUI)
    p.setGravity(0,0,-9.81)


    urdfRootPath=pybullet_data.getDataPath()
    kinovaUid = p.loadURDF('./j2s6s300.urdf',useFixedBase=True)

    # Dictionary of Robot Active Joints and Home Position
    arm_Joints = [2, 3, 4, 5, 6, 7]
    finger_joints = [9, 10, 11, 12, 13, 14]

    #homePos = [0.0, 2.9, 1.3, 4.2, 1.4, 0.0, 0, 0, 0]
    homePos = [math.pi, math.pi, math.pi, math.pi, math.pi, math.pi]

    homeStateDict = dict(zip(arm_Joints, homePos))

    # Sets robot home position before loading other objects
    for joint, pos in homeStateDict.items():
        p.resetJointState(kinovaUid, joint, pos)

    tableUid = p.loadURDF(os.path.join(urdfRootPath, "table/table.urdf"),basePosition=[0.5,0,-0.65])


    # Create a block collision shape
    blockShape = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.03, 0.03, 0.03])

    # Create a block rigid body
    blockBody = p.createMultiBody(baseMass=1.0, baseCollisionShapeIndex=blockShape)

    # Set the initial position and orientation of the block
    #initialPosition = [0.7, 0, 0.1]
    x_min, x_max = -0.2, 1.0  # Example limits for x coordinate
    y_min, y_max = -0.5, 0.5  # Example limits for y coordinate

    # Generate random x and y coordinates within the defined limits
    initial_x = np.random.uniform(x_min, x_max)
    initial_y = np.random.uniform(y_min, y_max)

    
    
    initialPosition = [initial_x, initial_y, 0.1]

    initialOrientation = p.getQuaternionFromEuler([0, 0, 0])  # no rotation
    p.resetBasePositionAndOrientation(blockBody, initialPosition, initialOrientation)

    objectUid = blockBody

    p.resetDebugVisualizerCamera(cameraDistance=1.5, cameraYaw=0, cameraPitch=-40, cameraTargetPosition=[0.55,-0.35,0.2])


    js_list = []


    state_durations = [0.25,0.5,0.25]
    control_dt = 1./240.
    p.setTimestep = control_dt
    state_t = 0.
    current_state = 0

    MAX_FINGER_POS = 1.5

    ikSolver = 0


    while runsim == True:
        p.stepSimulation()    
        
        js_list.append(current_joint_positions(kinovaUid, arm_Joints))

        state_t += control_dt
        p.configureDebugVisualizer(p.COV_ENABLE_SINGLE_STEP_RENDERING) 
        
        if current_state == 0:
            # Sets robot home position before loading other objects
            for joint, pos in homeStateDict.items():
                p.setJointMotorControl2(kinovaUid, joint, p.POSITION_CONTROL, pos)


            
        if current_state == 1:
            # Moves arm towards object, I can change the orientation at which the arm grabs the object if needed
            objPos = p.getBasePositionAndOrientation(objectUid)[0]


            jointPoses = p.calculateInverseKinematics(kinovaUid,
                                                            8,
                                                            objPos,
                                                            solver=ikSolver)

                
                
            for i in range(6):     
                
                p.setJointMotorControl2(bodyIndex=kinovaUid,
                                        jointIndex=i+2,
                                        controlMode=p.POSITION_CONTROL,
                                        targetPosition=jointPoses[i],
                                        targetVelocity=0,
                                        #force=500,
                                        positionGain=0.03,
                                        velocityGain=1
                                                        )
                                    

        if current_state == 2:
            # Closes the fingers to grab the object

            for i in finger_joints:
                if i % 2 == 1:
                    p.setJointMotorControl2(kinovaUid, i,
                                        p.POSITION_CONTROL, 0.6 * MAX_FINGER_POS, force = 200)
                    p.setJointMotorControl2(kinovaUid, i + 1,
                                        p.POSITION_CONTROL, 0.5 * MAX_FINGER_POS, force = 200)

            
        

        if state_t >state_durations[current_state]:
            current_state += 1
            if current_state >= len(state_durations):
                sim_results.append(js_list)
                p.disconnect()
                runsim = False
            state_t = 0

# Get the number of joints in the robot
numJoints = p.getNumJoints(kinovaUid)

# Iterate over each joint to get its information
for jointIndex in range(numJoints):
    jointInfo = p.getJointInfo(kinovaUid, jointIndex)
    jointName = jointInfo[1].decode("utf-8")  # Decode the joint name
    print("Joint Index:", jointIndex, "Joint Name:", jointName)

