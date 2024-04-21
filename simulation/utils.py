import pybullet as p
import numpy as np

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

def depth_to_pointcloud(depth, camera_intrinsics):
    """
    This function converts a depth image to a point cloud.
    
    Parameters:
    depth (np.ndarray): A 2D numpy array representing the depth image.
    camera_intrinsics (np.ndarray): A 1D numpy array representing the camera intrinsics.
    
    Returns:
    point_cloud (np.ndarray): A 2D numpy array representing the point cloud.
    """
    # Generate a grid of coordinates
    x = np.arange(0, depth.shape[1])
    y = np.arange(0, depth.shape[0])
    xx, yy = np.meshgrid(x, y)
    
    # Apply the camera intrinsics to the coordinates
    fx, fy, cx, cy = camera_intrinsics
    xx = (xx - cx) * depth / fx
    yy = (yy - cy) * depth / fy
    
    # Stack the coordinates to create the point cloud
    point_cloud = np.stack([xx, yy, depth], axis=-1)
    
    return point_cloud

def capture_point_cloud(camera_position, camera_orientation, object_uid):
    """
    Capture a point cloud from the specified camera position and orientation.
    """
    width, height = 640, 480
    view_matrix = p.computeViewMatrixFromYawPitchRoll(camera_position, distance=1, yaw=0, pitch=-10, roll=0, upAxisIndex=2)
    proj_matrix = p.computeProjectionMatrixFOV(fov=60, aspect=float(width)/height, nearVal=0.1, farVal=100)
    _, _, _, depthImg, segImg = p.getCameraImage(width, height, viewMatrix=view_matrix, projectionMatrix=proj_matrix)
    
    farVal = 100
    nearVal = 0.1
    depth = farVal * nearVal / (farVal - (farVal - nearVal) * depthImg)
    point_cloud = depth_to_pointcloud(depth, camera_position, camera_orientation)
    mask = (segImg == object_uid).astype(np.uint8)
    
    return point_cloud, mask