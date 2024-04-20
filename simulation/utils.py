import pybullet as p

def current_joint_positions(robot_id, joint_indices):
    """
    This function returns the current joint positions of a robot.
    
    Parameters:
    robot_id (int): The unique identifier for the robot.
    joint_indices (list): A list of indices representing the joints of the robot.
    
    Returns:
    current_joint_positions (list): A list of current positions for each joint specified by joint_indices.
    """
    current_joint_positions = []
    for joint_index in joint_indices:
        # getJointState returns a tuple where the first element is the current position of the joint
        joint_state = p.getJointState(robot_id, joint_index)
        current_joint_positions.append(joint_state[0])  # Append the joint position to the list
    
    return current_joint_positions