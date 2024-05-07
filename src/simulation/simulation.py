import pybullet as p
import pybullet_data
import os
import math
import numpy as np
from utils import current_joint_positions, get_point_cloud
import h5py
import tf

# Configuration
NUM_SIMS = 10
SIM_RESULTS = []
ARM_JOINTS = [2, 3, 4, 5, 6, 7]
FINGER_JOINTS = [9, 10, 11, 12, 13, 14]
HOME_POS = [math.pi, math.pi, math.pi/2, math.pi, math.pi/2, math.pi] #[math.pi] * len(ARM_JOINTS); Straight up
STATE_DURATIONS = [0.25, 1, 0.25, 0.25, 1, 0.25] 
CONTROL_DT = 1. / 2**9.
MAX_FINGER_POS = 1.5
GRASP_FORCE = 50
PID = [3 , 0.1, 0.1] # [Proportional Gain, Integral Gain, Derivative Gain]
MAX_VEL = 1

def setup_environment(with_gui):
    """
    Set up the PyBullet environment by loading the Kinova robot and a table.
    """
    urdf_root_path = pybullet_data.getDataPath()
    client_id = -1
    if with_gui:
        client_id = p.connect(p.GUI)
    else:
        client_id = p.connect(p.DIRECT)
    p.setGravity(0, 0, -9.81)
    p.setAdditionalSearchPath(urdf_root_path)
    p.resetDebugVisualizerCamera(3, 90, -30, [0.0, -0.0, -0.0])
    table_uid = p.loadURDF(os.path.join(urdf_root_path, "table/table.urdf"), basePosition=[0.5, 0, -0.65])
    kinova_uid = p.loadURDF("./robot/j2s6s300.urdf", useFixedBase=True)
    return kinova_uid, table_uid, client_id

def initialize_robot_position(kinova_uid):
    """
    Initialize the robot to the home position.
    """
    home_state_dict = dict(zip(ARM_JOINTS, HOME_POS))
    for joint, pos in home_state_dict.items():
        p.resetJointState(kinova_uid, joint, pos)

def create_object(num_object=1):
    """
    Create a block object in the environment.
    """
    block_shape = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.03, 0.03, 0.03])

    block_body = []
    for _ in range(num_object):
        block_body.append(p.createMultiBody(baseMass=1.0, baseCollisionShapeIndex=block_shape))
    
    #Min and max values represent workspace in which robot can perform the given task
    x_min, x_max = 0.1, 0.55
    y_min, y_max = -0.45, 0.45
    initial_x = np.random.uniform(x_min, x_max)
    initial_y = np.random.uniform(y_min, y_max)

    initial_position = [initial_x, initial_y, 0.1]
    initial_orientation = p.getQuaternionFromEuler([0, 0, 0])
    
    for i in range(num_object):
        p.changeDynamics(block_body[i], -1, lateralFriction=10, spinningFriction=0) # Adding friction to the block object for grabbing
        p.resetBasePositionAndOrientation(block_body[i], initial_position, initial_orientation)

        initial_position[2] += 0.06


    return block_body

def run_simulation(kinova_uid, object_uid, sim_type, client_id):
    """
    Simulation loop that runs for a fixed number of states.
    """
    js_list = []
    current_state = 0
    state_time = 0.

    if sim_type == 1:
        x_min, x_max = 0.1, 0.55
        y_min, y_max = -0.45, 0.45
        initial_x = np.random.uniform(x_min, x_max)
        initial_y = np.random.uniform(y_min, y_max)

        initial_position = [initial_x, initial_y, 0.1]
        initial_orientation = p.getQuaternionFromEuler([0, 0, 0])

        p.resetBasePositionAndOrientation(object_uid[-1], initial_position, initial_orientation)

    with h5py.File("sim_results.h5", "w") as hdf_file:
        while True:
            p.stepSimulation()
            if current_state == 0:
                    pc, mask = get_point_cloud()
                    hdf_file.create_dataset('point_cloud', data=pc)
                    hdf_file.create_dataset('mask', data=mask)
                    
            js_list.append(current_joint_positions(kinova_uid, ARM_JOINTS))
            hdf_file.create_dataset('joint_states', data=js_list)

            if sim_type == 0:
                robot_reach(kinova_uid, object_uid[-1], current_state)

            elif sim_type == 1 and len(object_uid)>2:
                robot_stack_XonY(kinova_uid, object_uid_x=object_uid[-1], object_uid_y=object_uid[-2], state=current_state)
            else:
                print('Not enough items or sim_type is out of range')
                return
            
            state_time += CONTROL_DT
            if state_time > STATE_DURATIONS[current_state]:
                current_state += 1
                if current_state >= len(STATE_DURATIONS):
                    SIM_RESULTS.append(js_list)
                    break
                elif sim_type == 0 and current_state > 2:
                    SIM_RESULTS.append(js_list)
                    break
                state_time = 0
            

def robot_reach(kinova_uid, object_uid, state):
    """
    Robot control logic for each state for the reach task
    """
    if state == 1:
        object_position = p.getBasePositionAndOrientation(object_uid)[0]
        target_position = [object_position[0], object_position[1], object_position[2]+0.01]
            
        #Desired end effector orientation, Has the arm grab the block from above    
        roll = 0
        pitch = math.pi
        yaw = 0

        quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
        target_orientation = [quaternion[0], quaternion[1], quaternion[2], quaternion[3]]

        current_positions = [p.getJointState(kinova_uid, i)[0] for i in ARM_JOINTS]
        joint_poses = p.calculateInverseKinematics(kinova_uid, 8, target_position, target_orientation, solver=0)
        position_error = [joint_poses[i] - current_positions[i] for i in range(len(current_positions))]
        targetVelocity = [i*PID[0] for i in position_error]

        for i in range(len(ARM_JOINTS)):
            p.setJointMotorControl2(kinova_uid, ARM_JOINTS[i], p.VELOCITY_CONTROL, targetVelocity = targetVelocity[i], maxVelocity = MAX_VEL)

    elif state == 2:
        for i in FINGER_JOINTS:
            if i % 2 == 1:
                p.setJointMotorControl2(kinova_uid, i,
                                    p.VELOCITY_CONTROL, targetVelocity = 2 ,force = GRASP_FORCE)
                p.setJointMotorControl2(kinova_uid, i + 1,
                                    p.VELOCITY_CONTROL, targetVelocity = 2 ,force = GRASP_FORCE)
                

def robot_stack_XonY(kinova_uid, object_uid_x, object_uid_y, state):
    """
    Robot control logic for each state for the stack skill
    """
    if state == 1:
        # Moves arm towards object
        objPos, objOr = p.getBasePositionAndOrientation(object_uid_x)


        handPos = [objPos[0], objPos[1], objPos[2]+0.015] #reaches right above top block
        handOr = p.getQuaternionFromEuler([0, -math.pi, 0]) # X, Y, Z

        # Get the current joint positions
        current_positions = [p.getJointState(kinova_uid, i)[0] for i in ARM_JOINTS]

        joint_poses = p.calculateInverseKinematics(kinova_uid, 8, handPos, handOr, solver=0)


        # The joint state is a tuple containing:
        # [0] - Joint position (in radians for revolute joints, in meters for prismatic joints)
        # [1] - Joint velocity
        # [2] - Joint reaction forces (torques for revolute joints, forces for prismatic joints)
        # [3] - Joint applied forces
            


        PosErr = [joint_poses[i] - current_positions[i] for i in range(len(current_positions))]

        targetVelocity = [i*PID[0] for i in PosErr]

        for i in range(len(ARM_JOINTS)):
            p.setJointMotorControl2(kinova_uid, ARM_JOINTS[i], p.VELOCITY_CONTROL, targetVelocity = targetVelocity[i], maxVelocity = MAX_VEL)


        
    if state == 2:
        for i in FINGER_JOINTS:
            if i % 2 == 1:
                p.setJointMotorControl2(kinova_uid, i,
                                    p.VELOCITY_CONTROL, targetVelocity = 2 ,force = GRASP_FORCE)
                p.setJointMotorControl2(kinova_uid, i + 1,
                                    p.VELOCITY_CONTROL, targetVelocity = 2 ,force = GRASP_FORCE)
                


    if state == 3:

        handPos = p.getLinkState(kinova_uid, 8)[0]

        handPos = [handPos[0], handPos[1], 0.3]


        handOr = p.getQuaternionFromEuler([0, -math.pi, 0]) # X, Y, Z

        # Get the current joint positions
        current_positions = [p.getJointState(kinova_uid, i)[0] for i in ARM_JOINTS]

        joint_poses = p.calculateInverseKinematics(kinova_uid,
                                                        8,
                                                        handPos, handOr,
                                                        solver=0)


        # The joint state is a tuple containing:
        # [0] - Joint position (in radians for revolute joints, in meters for prismatic joints)
        # [1] - Joint velocity
        # [2] - Joint reaction forces (torques for revolute joints, forces for prismatic joints)
        # [3] - Joint applied forces
            

        PosErr = [joint_poses[i] - current_positions[i] for i in range(len(current_positions))]

        targetVelocity = [i*PID[0] for i in PosErr]

        for i in range(len(ARM_JOINTS)):
            p.setJointMotorControl2(kinova_uid, ARM_JOINTS[i], p.VELOCITY_CONTROL, targetVelocity = targetVelocity[i], maxVelocity = MAX_VEL)

    if state == 4:
        
        # Moves arm towards object
        objPos, objOr = p.getBasePositionAndOrientation(object_uid_y)

        objInHandPos, _ = p.getBasePositionAndOrientation(object_uid_x)

        finalPos = [objPos[0], objPos[1], objPos[2]+0.1]

        # How far off center block is from being above the block
        blockPosErr = [finalPos[x] - objInHandPos[x] if abs(objPos[x] - objInHandPos[x]) < 0.1 else 0 for x in range(3)]

        # Should account for error if block isnt center in hand, Better PID tuning is needed for increased accuracy
        handPos = [objPos[0]+blockPosErr[0], objPos[1]+blockPosErr[1], objPos[2]+0.1+blockPosErr[2]] #places right on top block
        handOr = p.getQuaternionFromEuler([0, -math.pi, 0]) # X, Y, Z

        # Get the current joint positions
        current_positions = [p.getJointState(kinova_uid, i)[0] for i in ARM_JOINTS]

        joint_poses = p.calculateInverseKinematics(kinova_uid,
                                                        8,
                                                        handPos, handOr,
                                                        solver=0)


        PosErr = [joint_poses[i] - current_positions[i] for i in range(len(current_positions))]

        targetVelocity = [i*PID[0] for i in PosErr]

        for i in range(len(ARM_JOINTS)):
            p.setJointMotorControl2(kinova_uid, ARM_JOINTS[i], p.VELOCITY_CONTROL, targetVelocity = targetVelocity[i], maxVelocity = MAX_VEL)

    if state == 5:

        for i in FINGER_JOINTS:
            if i % 2 == 1:
                p.setJointMotorControl2(kinova_uid, i,
                                    p.POSITION_CONTROL, 0 * MAX_FINGER_POS)
                p.setJointMotorControl2(kinova_uid, i + 1,
                                    p.POSITION_CONTROL, 0 * MAX_FINGER_POS)


def simulate(num_sims, num_object, sim_type, with_gui=False):
    NUM_SIMS = num_sims
    for _ in range(NUM_SIMS):
        kinova_uid, _, client_id = setup_environment(with_gui)
        initialize_robot_position(kinova_uid)
        object_uid = create_object(num_object)
        run_simulation(kinova_uid, object_uid, sim_type, client_id)
        p.disconnect()

