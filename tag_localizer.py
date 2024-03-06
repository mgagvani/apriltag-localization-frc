"""
Use multiple AprilTags to estimate the pose of the camera

Utility for detect_apriltag.py
"""

import json
import numpy as np

from detect_apriltag import pose_to_transform
from utils import print_matrix

def load_tag_config(filename: str) -> dict:
    """
    Load the tag configuration from a SpectacularAI formatted JSON file.
    """
    with open(filename, 'r') as f:
        data = json.load(f)

    tags = {}

    for value in data:
        id = value['id']
        tagToWorld = value['tagToWorld'] # 4x4 matrix
        tags[id] = tagToWorld

    return tags

def np_to_list(np_array: np.ndarray) -> list[list[float]]:
    """
    Convert a numpy array to a list of lists
    """
    return [list(row) for row in np_array]

def get_global_pose(detections: list, tags: dict) -> np.ndarray:
    """
    Estimate the global field pose of the camera from the detected AprilTags
    Simple averaging of the poses of the detected tags
    
    Args:
    - detections (list): A list of AprilTag detections, each (id, pose) tuple
    - tags (dict): A dictionary of tag id to tagToWorld transform

    Returns:
    - np.ndarray: A 4x4 homogenous matrix representing the pose of the camera in the world frame
    """

    B = np.array([[0, 0, 1, 0],
                  [1, 0, 0, 0],
                  [0, -1, 0, 0],
                  [0, 0, 0, 1]])

    # get the pose of each tag
    tag_poses = []
    for detection in detections:
        id, pose = detection
        try:
            tagToWorld = tags[id] @ B # apply correction matrix for coordinate system
            tag_poses.append(tagToWorld @ pose) # transform the pose to the world frame
        except KeyError:
            print(f"Tag {id} not found in tag config")
            continue

    if len(tag_poses) == 0:
        return None

    # Separate rotation and translation components
    rotations = [pose[:3, :3] for pose in tag_poses]
    translations = [pose[:3, 3] for pose in tag_poses]

    # Average rotations and translations separately
    avg_rotation = np.mean(rotations, axis=0)
    avg_translation = np.mean(translations, axis=0)

    # Construct the averaged camera pose matrix
    camera_pose = np.eye(4)
    camera_pose[:3, :3] = avg_rotation
    camera_pose[:3, 3] = avg_translation

    return camera_pose

 
def main():
    tags = load_tag_config("apriltag_configs/crescendo/crescendo_apriltags.json")

    for id, tagToWorld in tags.items():
        print()
        print(f"Tag {id}:")
        print_matrix(tagToWorld)

    fake_camera_pose = np.eye(4)
    fake_camera_pose[:3, 3] = np.array([0, 0.5, 3])

    # print(tags[3] @ fake_camera_pose)


if __name__ == "__main__":
    main()



