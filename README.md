# AprilTag-Based Localization for FRC

## Installation
1. Install OpenCV, NumPy and Matplotlib. 
2. Install DepthAI and Spectacular AI dependencies: `pip install depthai spectacularai --force`. See https://github.com/SpectacularAI/sdk-examples.
3. Test Spectacular AI by running `vio_visu.py` from their repo ([link](https://github.com/SpectacularAI/sdk-examples/blob/main/python/oak/vio_visu.py))

## Notes
* Tested with the Python 3.10 venv bundled with the DepthAI Demo: ([link](https://docs.luxonis.com/en/latest/pages/tutorials/first_steps/))
* I'm not sure if the `quaternion_to_theta()` and `quaternion_to_theta_xyz_planes()` functions are actually correct.
* You cannot serialize a Pose object. It also has no default constructor.
* Converting the homogenous matrix's rotation matrix to Euler angles gives you yaw. This is definetly correct. 

## Testing
* The VIO only works if there is an AprilTag present when the program starts running.
* Notes from `test_apriltags.json`, which has one vertically oriented AprilTag at the origin
  - Moving right increases X
  - Since the tag is at the origin, Y is negative when you're looking at it and moving farther from the tag makes Y more negative
  - Moving up increases Z
* Yaw from Euler angles is the angle we will send over NetworkTables
  - This is because the angles are X, Y, Z --> Roll, Pitch, Yaw, and we are rotating about the Z-axis

## Goals:
- [x] Use Spectacular AI SDK to get pose of camera relative to AprilTag(s)
- [ ] Verify accuracy of localization with:
  - [x] No AprilTag
  - [x] One AprilTag
  - [ ] Multiple AprilTags
- [x] Convert data into format usable by FRC's Pose2d object (x, y, theta) from quaternions
- [ ] Stream data over NetworkTables
- [ ] Use streamed data in place of Pigeon/NavX/Dead Reckoning on the robot