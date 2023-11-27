# AprilTag-Based Localization for FRC

## Installation
1. Install OpenCV, NumPy and Matplotlib. 
2. Install DepthAI and Spectacular AI dependencies: `pip install depthai spectacularai --force`. See https://github.com/SpectacularAI/sdk-examples.
3. Test Spectacular AI by running `vio_visu.py` from their repo ([link](https://github.com/SpectacularAI/sdk-examples/blob/main/python/oak/vio_visu.py))

## Notes
* I'm not sure if the `quaternion_to_theta()` and `quaternion_to_theta_xyz_planes()` functions are actually correct.

## Testing
* The VIO only works if there is an AprilTag present when the program starts running.
* Notes from `test_apriltags.json`, which has one vertically oriented AprilTag at the origin
  - Moving right increases X
  - Since the tag is at the origin, Y is negative when you're looking at it and moving farther from the tag makes Y more negative
  - Moving up increases Z

## Goals:
- [x] Use Spectacular AI SDK to get pose of camera relative to AprilTag(s)
- [ ] Verify accuracy of localization with no/single/multiple AprilTags
- [ ] Convert data into format usable by FRC's Pose2d object (x, y, theta) from quaternions
- [ ] Use streamed data in place of Pigeon/NavX/Dead Reckoning on the robot