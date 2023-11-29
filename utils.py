"""
Convert a quaternion (w, x, y, z) to a 2D theta value.
Pose2D objects in WPILib use a 2D theta value.
But SpectacularAI uses a quaternion (w, x, y, z) for its pose.
"""
import numpy as np
import logging
class CustomLogger:
    def __init__(self, log_file='logs/oakd.log', debug_to_stdout=True):
        # Configure the logging
        logging.basicConfig(level=logging.DEBUG,
                            format='%(asctime)s.%(msecs)03d [%(levelname)s] %(message)s',
                            datefmt='%Y-%m-%d %H:%M:%S')

        # Create a file handler for INFO logs
        info_handler = logging.FileHandler(log_file, mode='a') # append to the log file
        info_handler.setLevel(logging.INFO)
        info_handler.setFormatter(logging.Formatter('%(asctime)s.%(msecs)03d [%(levelname)s] %(message)s',
                                                    datefmt='%Y-%m-%d %H:%M:%S'))

        # Add the INFO handler to the logger
        self.logger = logging.getLogger('')
        self.logger.addHandler(info_handler)

        # Set whether DEBUG logs should go to stdout
        debug_handler = logging.StreamHandler() if debug_to_stdout else logging.NullHandler()
        debug_handler.setLevel(logging.DEBUG)
        debug_handler.setFormatter(logging.Formatter('%(asctime)s.%(msecs)03d [%(levelname)s] %(message)s',
                                                      datefmt='%Y-%m-%d %H:%M:%S'))

        # Add the DEBUG handler to the logger
        self.logger.addHandler(debug_handler)

    def log_debug(self, message):
        '''
        Log a debug message (only if debug_to_stdout is True).
        '''
        self.logger.debug(message)

    def log_info(self, message):
        '''
        Log an info message (goes to log file only)
        '''
        self.logger.info(message)

def quaternion_to_theta(quaternion):
    """
    Convert a quaternion (w, x, y, z) to a 2D theta value.

    Parameters:
    - quaternion (list or numpy array): The quaternion in the order (w, x, y, z).

    Returns:
    - float: The 2D theta value in radians.
    """
    # Ensure the quaternion is a numpy array for vector operations
    quaternion = np.array(quaternion)

    # Extract the rotation axis (x, y, z) from the quaternion
    rotation_axis = quaternion[1:]

    # Compute the angle of rotation (theta) using the arctan2 function
    theta = 2 * np.arctan2(np.linalg.norm(rotation_axis), quaternion[0])

    return theta

def quaternion_to_theta_xyz_planes(quaternion):
    """
    Convert a quaternion (w, x, y, z) to 2D theta values for rotation in XY, XZ, and YZ planes.

    Parameters:
    - quaternion (list or numpy array): The quaternion in the order (w, x, y, z).

    Returns:
    - dict: A dictionary containing theta values for rotation in XY, XZ, and YZ planes.
    """
    # Ensure the quaternion is a numpy array for vector operations
    quaternion = np.array(quaternion)

    # Extract the rotation axes (x, y, z) from the quaternion
    rotation_axes = quaternion[1:]

    # Compute theta values for rotation in XY, XZ, and YZ planes
    theta_xy = np.arctan2(rotation_axes[1], rotation_axes[0])
    theta_xz = np.arctan2(rotation_axes[2], rotation_axes[0])
    theta_yz = np.arctan2(rotation_axes[2], rotation_axes[1])

    # Create and return a dictionary with the theta values
    theta_values = {'XY': theta_xy, 'XZ': theta_xz, 'YZ': theta_yz}
    return theta_values

def homogenous_to_euler(homogenous_matrix):
    """
    Convert a homogenous transformation matrix to euler angles.

    Parameters:
    - homogenous_matrix (numpy array): The homogenous transformation matrix.

    Returns:
    - list: A list of euler angles in the order (roll, pitch, yaw).
    """
    # Ensure the homogenous matrix is a numpy array for vector operations
    homogenous_matrix = np.array(homogenous_matrix)

    # Extract the rotation matrix from the homogenous matrix
    rotation_matrix = homogenous_matrix[:3, :3]

    # Extract the translation vector from the homogenous matrix
    translation_vector = homogenous_matrix[:3, 3]

    # Compute the euler angles using the rotation matrix
    euler_angles = np.array([0.0, 0.0, 0.0])

    # Roll (rotation about x-axis)
    euler_angles[0] = np.arctan2(rotation_matrix[2, 1], rotation_matrix[2, 2])

    # Pitch (rotation about y-axis)
    euler_angles[1] = np.arctan2(-rotation_matrix[2, 0], np.sqrt(rotation_matrix[2, 1] ** 2 + rotation_matrix[2, 2] ** 2))

    # Yaw (rotation about z-axis)
    euler_angles[2] = np.arctan2(rotation_matrix[1, 0], rotation_matrix[0, 0])

    return euler_angles

def round_list(list, digits=4):
    """
    Utility function: Round a list of numbers with a specified number of digits.

    Parameters:
    - list (list): The list of numbers to print.
    - digits (int): The number of digits to print.
    """
    rounded_list = [round(x, digits) for x in list]

    return rounded_list

def wrap_to_pi(theta):
    """
    Utility function: Wrap a theta value to the range [-pi, pi].

    Parameters:
    - theta (float): The theta value to wrap.

    Returns:
    - float: The wrapped theta value.
    """
    return (theta + np.pi) % (2 * np.pi) - np.pi

def wrap_to_2pi(theta):
    """
    Utility function: Wrap a theta value to the range [0, 2pi].

    Parameters:
    - theta (float): The theta value to wrap.

    Returns:
    - float: The wrapped theta value.
    """
    return (theta + 2 * np.pi) % (2 * np.pi)

def print_matrix(matrix, digits=4):
    """
    Utility function: Print a matrix with a specified number of digits.

    Parameters:
    - matrix (numpy or 2D list of lists): The matrix to print.
    - digits (int): The number of digits to print.
    """
    # Round the matrix to the specified number of digits
    rounded_matrix = [[round(x, digits) for x in row] for row in matrix]

    # find number of characters needed to print matrix
    lengths = [[len(str(x)) for x in row] for row in rounded_matrix]
    max_lengths = [max(x) for x in zip(*lengths)]
    
    padding = "-" * (sum(max_lengths) + len(lengths) - 1)

    print(padding)
    print('\n'.join(['\t'.join([str(cell) for cell in row]) for row in rounded_matrix]))

if __name__ == "__main__":
    # Test the function with a sample quaternion
    quaternion = np.array([0.7071, 0.0, 0.0, 0.7071])

    theta = quaternion_to_theta(quaternion)
    theta_degrees = np.rad2deg(theta)
    print(f"2D Theta value: {theta_degrees} degrees")

    theta_values = quaternion_to_theta_xyz_planes(quaternion)
    print(f"XY Theta value: {np.rad2deg(theta_values['XY'])} degrees")
    print(f"XZ Theta value: {np.rad2deg(theta_values['XZ'])} degrees")
    print(f"YZ Theta value: {np.rad2deg(theta_values['YZ'])} degrees")
