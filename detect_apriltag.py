"""
AprilTag Detection using pupil-apriltags
"""

import cv2
import numpy as np
import pupil_apriltags as apriltag
import depthai
import atexit
from time import perf_counter
from utils import *

from networktables import NetworkTables

PRINT_LOGS = True

NetworkTables.initialize(server="10.85.92.2")
sd = NetworkTables.getTable("SmartDashboard")
sd.putNumber("jetson_apriltag_x", 0)
sd.putNumber("jetson_apriltag_y", 0)
sd.putNumber("jetson_apriltag_z", 0)
sd.putNumber("jetson_apriltag_id", 0)

# todo figure out what imu data is returned
sd.putNumber("jetson_apriltag_imu_x", 0)
sd.putNumber("jetson_apriltag_imu_y", 0)
sd.putNumber("jetson_apriltag_imu_z", 0)

sd.putBoolean("jetson_active", True)

def exit_handler():
    sd.putBoolean("jetson_active", False)

atexit.register(exit_handler)

def get_camera_params():
    """
    Get mono camera parameters @ 800p

    Returns (fx, fy, cx, cy) of mono camera
    """
    with depthai.Device() as device:
        calibData = device.readCalibration()

        # intrinsics are 3x3 matrix
        intrinsics = calibData.getCameraIntrinsics(depthai.CameraBoardSocket.RIGHT)
        fx, fy, cx, cy = intrinsics[0][0], intrinsics[1][1], intrinsics[0][2], intrinsics[1][2]
    return fx, fy, cx, cy

def pose_to_transform(poseR, pose_t):
    rotationAndTranslation = np.concatenate((poseR, pose_t), 1)
    return np.concatenate((rotationAndTranslation, np.array([[0, 0, 0 , 1]])), 0)
        

def main():
    # camera parameters and apriltag detector
    camera_params = get_camera_params()
    detector = apriltag.Detector(
        families="tag36h11",
        nthreads=4,
    )

    # get mono camera
    pipeline = depthai.Pipeline()
    mono = pipeline.create(depthai.node.MonoCamera)
    mono.setCamera("right")
    mono.setResolution(depthai.MonoCameraProperties.SensorResolution.THE_800_P)
    mono.setFps(120)

    # create output queue for mono camera
    monoOut = pipeline.create(depthai.node.XLinkOut)
    monoOut.setStreamName("mono")
    mono.out.link(monoOut.input)

    # IMU
    # imu = pipeline.create(depthai.node.IMU)
    # imu.enableIMUSensor([depthai.IMUSensor.ARVR_STABILIZED_ROTATION_VECTOR], 100)
    # imuOut = pipeline.create(depthai.node.XLinkOut)
    # imuOut.setStreamName("imu")
    # imu.out.link(imuOut.input)

    # logger
    cLogger = CustomLogger(debug_to_stdout=PRINT_LOGS)

    # loop
    with depthai.Device(pipeline) as device:
        # output queue
        q = device.getOutputQueue(name="mono", maxSize=4, blocking=False)
        # imuQueue = device.getOutputQueue(name="imu", maxSize=4, blocking=False)
        counter, timestamps = 0, []

        while True:
            counter += 1
            timestamps.append(perf_counter())

            # get imu
            # imuData = imuQueue.get()
            # imuData = imuData.packets[0].rotationVector # TODO: this has .i, .j, .k, .real, convert to usable data
            # print(imuData)
            # sd.putNumber("jetson_apriltag_imu_x", imuData.x)
            # sd.putNumber("jetson_apriltag_imu_y", imuData.y)
            # sd.putNumber("jetson_apriltag_imu_z", imuData.z)

            inFrame = q.get()
            frame = inFrame.getCvFrame()
            # gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            # detect apriltag
            detections = detector.detect(frame, estimate_tag_pose=True, camera_params=camera_params, tag_size=0.1651)
            if len(detections) > 0:
                # get pose matrix of first detection
                detection = detections[0]
                poseR = detection.pose_R
                pose_t = detection.pose_t

                pose = pose_to_transform(poseR, pose_t)
                x, y, z = pose[0, 3], pose[1, 3], pose[2, 3]
                cLogger.log_debug(f"AprilTag {detection.tag_id} x, y, z: ({x}, {y}, {z})")
                sd.putNumber("jetson_apriltag_x", x)
                sd.putNumber("jetson_apriltag_y", y)
                sd.putNumber("jetson_apriltag_z", z)

                sd.putNumber("jetson_apriltag_id", detection.tag_id)

            # show frame
            # cv2.imshow("frame", frame)
            # if cv2.waitKey(1) == ord("q"):
            #     break
            if counter % 100 == 0:
                cLogger.log_info("FPS (last 100 frames): {}".format(100 / (timestamps[-1] - timestamps[0])))
                counter, timestamps = 0, [timestamps[-1]]
            

    cv2.destroyAllWindows()





if __name__ == "__main__":
    main()


