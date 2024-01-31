# AprilTag-Based Localization for FRC

## Installation
1. Install OpenCV, NumPy and Matplotlib. 
2. Install DepthAI and Spectacular AI dependencies: `pip install depthai spectacularai --force`. See https://github.com/SpectacularAI/sdk-examples.
3. Install pupil_apriltags: `pip install pupil-apriltags`
4. Test Spectacular AI by running `vio_visu.py` from their repo ([link](https://github.com/SpectacularAI/sdk-examples/blob/main/python/oak/vio_visu.py))

## Usage
1. The Jetson must be connected over Ethernet to the switch. The Jetson and OAK should be getting power over separate buck converters.
2. To access the Jetson, turn on the robot, and connect to its Wi-Fi. Run `ssh newton2@10.85.92.30` and type in the password (same as our Driver Station).
3. Navigate to `april-tags-experiment/vio_slam/apriltag-localization-frc/`. You can run either `python3.7 detect_apriltag.py` or `python3.7 slam_apriltag.py`.

## Notes
* Tested with the Python 3.10 venv bundled with the DepthAI Demo: ([link](https://docs.luxonis.com/en/latest/pages/tutorials/first_steps/)) and Python 3.7 on the Jetson
* I'm not sure if the `quaternion_to_theta()` and `quaternion_to_theta_xyz_planes()` functions are actually correct.
* You cannot serialize a SpectacularAI Pose object. It also has no default constructor.
* Converting the homogenous matrix's rotation matrix to Euler angles gives you yaw. This is definetly correct. 

## Testing
* The VIO only works if there is an AprilTag present when the program starts running.
* Notes from `test_apriltags.json`, which has one vertically oriented AprilTag at the origin
  - Moving right increases X
  - Since the tag is at the origin, Y is negative when you're looking at it and moving farther from the tag makes Y more negative
  - Moving up increases Z
* Yaw from Euler angles is the angle we will send over NetworkTables
  - This is because the angles are X, Y, Z --> Roll, Pitch, Yaw, and we are rotating about the Z-axis
* The camera must be rotated about its center or its position readings become slightly inaccurate
* Notes from `detect_apriltag.py` which is camera-relative AprilTag detection 
  - x and y are in the plane of the camera's lens, and z points out from the camera
  - Using an 800p stream, we successfully detected AprilTags up to about ~7m, but it was more inconsistent as the camera was moved farther
  - FPS for the above test was varied from 20-30 but generally stayed at about 26


## Goals:
- [x] Use Spectacular AI SDK to get pose of camera relative to AprilTag(s)
- [ ] Verify accuracy of localization with:
  - [x] No AprilTag
  - [x] One AprilTag
  - [x] Multiple AprilTags
- [x] Convert data into format usable by FRC's Pose2d object (x, y, theta) from quaternions
- [x] Use logging instead of prints
- [x] Stream data over NetworkTables
- [ ] Use streamed data in place of Pigeon/NavX/Dead Reckoning on the robot
- [x] Get AprilTag position/heading relative to the camera