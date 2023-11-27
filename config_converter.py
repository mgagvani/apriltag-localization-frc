"""
Script to turn WPILib-style AprilTag JSON to SpectacularAI-style JSON
Converts the quaternions to homogenous matrices
"""

import spectacularAI
import json
import pickle
import functools

from utils import *

# lru_cache is used to cache the blank pose object 
# so it doesn't have to be loaded from disk every time
@functools.lru_cache(maxsize=1)
def blank_pose_loader(path="apriltag_configs/pose_obj.pkl") -> spectacularAI.Pose:
    '''
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


def convert_apriltag(input_path):
    # read JSON from file
    with open(input_path, 'r') as f:
        json_data = json.load(f)

    output = {}

    tag_data = json_data['tags']
    for tag in tag_data:
        id = tag['ID']
        pose_dict = tag['pose']
        pose_obj = construct_pose_from_dict(pose_dict)
        matrix = pose_obj.asMatrix()
        print_matrix(matrix)
        input()

if __name__ == "__main__":
    input_path = "apriltag_configs/bunnybots/Bunnybots_2023.json"
    output_path = "apriltag_configs/bunnybots/bunnybots_apriltags.json"
    convert_apriltag(input_path)

