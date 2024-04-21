import pybullet as p
import pybullet_data
import os
import math
import numpy as np
from utils import current_joint_positions, capture_point_cloud
import h5py

# Configuration
NUM_SIMS = 10
SIM_RESULTS = []
ARM_JOINTS = [2, 3, 4, 5, 6, 7]
FINGER_JOINTS = [9, 10, 11, 12, 13, 14]
HOME_POS = [math.pi] * len(ARM_JOINTS)
STATE_DURATIONS = [0.25, 0.5, 0.25]
CONTROL_DT = 1. / 240.
MAX_FINGER_POS = 1.5

def setup_environment(with_gui):
    """
    Set up the PyBullet environment by loading the Kinova robot and a table.
    """
    urdf_root_path = pybullet_data.getDataPath()
    if with_gui:
        p.connect(p.GUI)
    else:
        p.connect(p.DIRECT)
    p.setGravity(0, 0, -9.81)
    p.setAdditionalSearchPath(urdf_root_path)
    table_uid = p.loadURDF(os.path.join(urdf_root_path, "table/table.urdf"), basePosition=[0.5, 0, -0.65])
    kinova_uid = p.loadURDF("./robot/j2s6s300.urdf", useFixedBase=True)
    return kinova_uid, table_uid

def initialize_robot_position(kinova_uid):
    """
    Initialize the robot to the home position.
    """
    home_state_dict = dict(zip(ARM_JOINTS, HOME_POS))
    for joint, pos in home_state_dict.items():
        p.resetJointState(kinova_uid, joint, pos)

def create_object():
    """
    Create a block object in the environment.
    """
    block_shape = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.03, 0.03, 0.03])
    block_body = p.createMultiBody(baseMass=1.0, baseCollisionShapeIndex=block_shape)
    x_min, x_max = -0.2, 1.0
    y_min, y_max = -0.5, 0.5
    initial_x = np.random.uniform(x_min, x_max)
    initial_y = np.random.uniform(y_min, y_max)
    initial_position = [initial_x, initial_y, 0.1]
    initial_orientation = p.getQuaternionFromEuler([0, 0, 0])
    p.resetBasePositionAndOrientation(block_body, initial_position, initial_orientation)
    return block_body

def run_simulation(kinova_uid, object_uid):
    """
    Simulation loop that runs for a fixed number of states.
    """
    js_list = []
    current_state = 0
    state_time = 0.
    with h5py.File("sim_results.h5", "w") as hdf_file:
        while True:
            p.stepSimulation()
            if current_state == 0:  # Capture point cloud at the beginning
                    pc, mask = capture_point_cloud([1, 1, 1], [0, 0, 0])  # Adjust camera position/orientation
                    hdf_file.create_dataset('point_cloud', data=pc)
                    hdf_file.create_dataset('mask', data=mask)
                    
            js_list.append(current_joint_positions(kinova_uid, ARM_JOINTS))
            hdf_file.create_dataset('joint_states', data=js_list)
            
            state_time += CONTROL_DT
            if state_time > STATE_DURATIONS[current_state]:
                current_state += 1
                if current_state >= len(STATE_DURATIONS):
                    SIM_RESULTS.append(js_list)
                    break
                state_time = 0
            control_robot_state(kinova_uid, object_uid, current_state)

def control_robot_state(kinova_uid, object_uid, state):
    """
    Robot control logic for each state.
    """
    if state == 1:
        target_position = p.getBasePositionAndOrientation(object_uid)[0]
        joint_poses = p.calculateInverseKinematics(kinova_uid, 8, target_position)
        for i, pos in enumerate(joint_poses):
            if i < len(ARM_JOINTS):
                p.setJointMotorControl2(kinova_uid, ARM_JOINTS[i], p.POSITION_CONTROL, pos)
    elif state == 2:
        for i, joint in enumerate(FINGER_JOINTS):
            target_pos = 0.6 * MAX_FINGER_POS if i % 2 == 0 else 0.5 * MAX_FINGER_POS
            p.setJointMotorControl2(kinova_uid, joint, p.POSITION_CONTROL, target_pos, force=200)

def simulate(num_sims, with_gui=False):
    NUM_SIMS = num_sims
    for _ in range(NUM_SIMS):
        kinova_uid, _ = setup_environment(with_gui)
        initialize_robot_position(kinova_uid)
        object_uid = create_object()
        run_simulation(kinova_uid, object_uid)
        p.disconnect()
