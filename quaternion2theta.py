"""
Convert a quaternion (w, x, y, z) to a 2D theta value.
Pose2D objects in WPILib use a 2D theta value.
But SpectacularAI uses a quaternion (w, x, y, z) for its pose.
"""
import numpy as np

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

if __name__ == "__main__":
    # Test the function with a sample quaternion
    quaternion = np.array([0.7071, 0.0, 0.0, 0.7071])

    theta = quaternion_to_theta(quaternion)
    theta_degrees = np.rad2deg(theta)
    print(f"2D Theta value: {theta_degrees} degrees")
