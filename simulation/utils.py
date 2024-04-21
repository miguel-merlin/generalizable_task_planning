import pybullet as p
import numpy as np
import pdb

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


def capture_point_cloud(obj_pb_id, width, height, projection_matrix, process_id):
    far, near = 1.1, 0.1
    pc = []
    normals = []
    camera_angles = [(1, 10), (45, 45), (90, 90), (135, 135), (179, 180)]
    
    for phi_deg, theta_deg in camera_angles:
        camera_distance = np.random.uniform(0.35, 0.55)
        phi = np.deg2rad(phi_deg)
        theta = np.deg2rad(theta_deg)
        obj_center = np.array(p.getBasePositionAndOrientation(obj_pb_id, physicsClientId=process_id)[0])
        camera_eye_position = obj_center + camera_distance * np.array([np.cos(phi) * np.sin(theta), np.sin(phi) * np.sin(theta), np.cos(theta)])
        
        view_matrix = p.computeViewMatrix(cameraEyePosition=list(camera_eye_position),
                                          cameraTargetPosition=list(obj_center),
                                          cameraUpVector=[0, 0, 1],
                                          physicsClientId=process_id)
        
        images = p.getCameraImage(width, height, viewMatrix=view_matrix, projectionMatrix=projection_matrix, renderer=p.ER_TINY_RENDERER, physicsClientId=process_id)
        depth_buffer = np.reshape(images[3], (height, width))
        depth = far * near / (far - (far - near) * depth_buffer)
        
        # Convert depth buffer to point cloud and calculate normals
        for u in range(height):
            for v in range(width):
                if depth[u, v] > 0:  # Simple segmentation based on depth threshold
                    point_world = project_pixel_to_point(u, v, depth[u, v], projection_matrix, view_matrix)
                    pc.append(point_world[:3])
    
    return pc, normals

def project_pixel_to_point(u, v, depth, projection_matrix, view_matrix):
    width, height = 640, 480
    
    projection_matrix_np = np.reshape(np.array(projection_matrix), (4, 4)).T
    view_matrix_np = np.reshape(np.array(view_matrix), (4, 4)).T
    inv_projection_matrix = np.linalg.inv(projection_matrix_np)
    inv_view_matrix = np.linalg.inv(view_matrix_np)

    # Convert pixel coordinates (u, v) to normalized device coordinates (NDC)
    x = (2.0 * v / width - 1.0)
    y = (2.0 * u / height - 1.0)
    z = 2.0 * depth - 1.0
    point_ndc = np.array([x, y, z, 1.0])
    point_camera = np.dot(inv_projection_matrix, point_ndc)
    point_camera /= point_camera[3]
    point_world = np.dot(inv_view_matrix, point_camera)
    point_world /= point_world[3]

    # Return only the x, y, z components
    return point_world[:3]  
                   
                   
def get_point_cloud(object_uid, process_id):
    """
    Capture a point cloud from the specified camera position and orientation.
    """
    width, height = 640, 480
    f_x, f_y = 543.820, 546.691
    c_x, c_y = 314.424, 237.466
    skew = 0.283204
    far, near = 1.1, 0.1
    
    opengl_projection_matrix = [f_x/width, 0, 0, 0,
                            skew/width, f_y/height, 0, 0,
                            2*(c_x+0.5)/width-1, 2*(c_y+0.5)/height-1, -(far+near)/(far-near), -1,
                            0, 0, -2*far*near/(far-near), 0]
    
    pc, normals = capture_point_cloud(object_uid, width, height, opengl_projection_matrix, process_id)
    
