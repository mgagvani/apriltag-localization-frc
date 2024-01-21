"""
Script to turn WPILib-style AprilTag JSON to SpectacularAI-style JSON
Converts the quaternions to homogenous matrices
"""

import spectacularAI
from scipy.spatial.transform import Rotation as R
import json
import pickle
import functools

from utils import *

# lru_cache is used to cache the blank pose object 
# so it doesn't have to be loaded from disk every time
@functools.lru_cache(maxsize=1)
def blank_pose_loader(path="apriltag_configs/pose_obj.pkl") -> spectacularAI.Pose:
    '''
    NOTE: This function is useless because you can't pkl it...
    
    Since spectacularAI.Pose has no default constructor
    we need to load a saved one from a pkl

    Parameters:
    - path (str): The path to the pickle file containing the pose object

    Returns:
    - spectacularAI.Pose: A SpectacularAI Pose object with all values set to 0
    '''
    with open(path, 'rb') as f:
        pose_obj = pickle.load(f)
    
    # zero it out
    for key in "xyz":
        setattr(pose_obj.position, key, 0)
    
    for key in "wxyz":
        setattr(pose_obj.orientation, key, 0)

    return pose_obj

def construct_pose_from_dict(pose_dict) -> spectacularAI.Pose:
    """
    NOTE: This function is similarly useless because Pose has no default constructor...

    Construct a SpectacularAI Pose object from a dictionary.

    Parameters:
    - pose_dict (dict): A dictionary containing the pose data.

    Returns:
    - spectacularAI.Pose: A SpectacularAI Pose object.
    """
    # Extract the position and orientation from the dictionary
    print(pose_dict)
    position = pose_dict['translation']
    orientation = pose_dict['rotation']['quaternion']

    # Create a SpectacularAI Pose object
    pose = spectacularAI.Pose()
    for key in "xyz":
        setattr(pose.position, key, position[key])

    for key in "WXYZ":
        # spectacularAI wants lowercase keys but JSON has uppercase for the quaternion
        setattr(pose.orientation, key.lower(), orientation[key])

def pose_to_matrix(pose_dict) -> np.ndarray:
    '''
    Convert a pose dictionary to a homogenous matrix using Scipy

    Parameters:
    - pose_dict (dict): A dictionary containing the pose data.

    Returns:
    - np.ndarray: A 4x4 homogenous matrix
    '''
    # Extract the position and orientation from the dictionary
    print(pose_dict)
    position = pose_dict['translation']
    orientation = pose_dict['rotation']['quaternion']

    x, y, z = position['x'], position['y'], position['z']
    i, j, k, w = orientation['X'], orientation['Y'], orientation['Z'], orientation['W']

    # create rotation matrix
    r = R.from_quat([i, j, k, w])
    r_matrix = r.as_matrix()

    # create homogenous matrix
    matrix = np.eye(4) # 4x4 identity matrix
    matrix[:3, :3] = r_matrix # first 3 in rows and columns
    matrix[:3, 3] = [x, y, z] # first 3 in rows, last in columns

    return matrix

def convert_fmap(input_path):
    with open(input_path, 'r') as f:
        json_data = json.load(f)

    output = {}
    tag_data = json_data['fiducials']
    for tag in tag_data:
        id = tag['id']
        size = tag['size'] / 1000 # convert to meters
        raw_transform = tag['transform'] # array of 16 floats
        matrix = np.array(raw_transform).reshape((4, 4))
        print(id)
        print_matrix(matrix); print()
        output[id] = matrix

    return output


def convert_apriltag(input_path):
    # read JSON from file
    with open(input_path, 'r') as f:
        json_data = json.load(f)

    output = {}

    tag_data = json_data['tags']
    for tag in tag_data:
        id = tag['ID']
        pose_dict = tag['pose']
        # pose_obj = construct_pose_from_dict(pose_dict)
        matrix = pose_to_matrix(pose_dict)
        print_matrix(matrix); print()
        output[id] = matrix
    
    return output

def viz_apriltags(output):
    '''
    Visualize the AprilTags in 3D space using Matplotlib

    Parameters:
    - output (dict): A dictionary containing the homogenous matrices
    '''
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d import Axes3D
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    VIEW_SIZE = 0.2 # meters
    CORNERS = np.array([
        [-VIEW_SIZE, -VIEW_SIZE, 0],
        [-VIEW_SIZE, VIEW_SIZE, 0],
        [VIEW_SIZE, VIEW_SIZE, 0],
        [VIEW_SIZE, -VIEW_SIZE, 0]
    ])

    for id, matrix in output.items():
        # draw square with correct orientation using Poly3DCollection
        # get the corners
        tag_corners = CORNERS @ matrix[:3, :3].T # rotate the corners
        tag_corners += matrix[:3, 3] # translate the corners
        
        # draw the square
        tag = Poly3DCollection([tag_corners], alpha=0.5, edgecolors='k')
        # color based on id
        tag._facecolors2d = tag._facecolor3d
        tag._edgecolors2d = tag._edgecolor3d
        tag.set(facecolors=plt.cm.tab10(int(id) % 10))
        # label the square
        ax.text(*tag_corners[0], id, fontsize=8)
        ax.add_collection3d(tag)

    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_xlim(-10, 10)
    ax.set_ylim(-10, 10)
    ax.set_zlim(0, 10)
    ax.set_title('AprilTags')

    plt.show()

def export_to_json(output_path, output, family='tag36h11', size=0.152) -> None:
    '''
    Export the homogenous matrices to a JSON file for use in SpectacularAI

    Parameters:
    - output_path (str): The path to the output JSON file
    - output (dict): A dictionary containing the homogenous matrices
    - family (str): The family of AprilTags
    - size (float): The size of the AprilTags
    '''
    # create the output dictionary
    output_list = []
    
    for id, matrix in output.items():
        tag_dict = {}
        tag_dict['id'] = id
        tag_dict['size'] = size
        tag_dict['family'] = family
        tag_dict['tagToWorld'] = matrix.tolist()
        output_list.append(tag_dict)

    # write to file
    with open(output_path, 'w') as f: # overwrite
        json.dump(output_list, f, indent=4)

def main_bunnybots():
    input_path = "apriltag_configs/bunnybots/Bunnybots_2023.json"
    output_path = "apriltag_configs/bunnybots/bunnybots_apriltags.json"
    converted_tag_poses = convert_apriltag(input_path)
    export_to_json(output_path, converted_tag_poses)

def main_fmap():
    input_path = "apriltag_configs/crescendo/unofficial_2024.fmap"
    tag_poses = convert_fmap(input_path)
    export_to_json("apriltag_configs/crescendo/crescendo_apriltags.json", tag_poses, family='tag36h11', size=0.1651)
    viz_apriltags(tag_poses)

if __name__ == "__main__":
    main_fmap()

