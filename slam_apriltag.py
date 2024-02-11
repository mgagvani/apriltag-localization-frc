"""
AprilTag SLAM using DepthAI and SpectacularAI.
Includes live camera preview (can disable).
Requirements: pip install opencv-python
"""
import depthai
import time
import spectacularAI
import threading
import numpy as np
import logging
from networktables import NetworkTables

from utils import *
from detect_apriltag import *
from tag_localizer import *

SHOW_CAM = False
if SHOW_CAM:
    import cv2

PRINT_LOGS = True

def make_pipelines(aprilTagPath="apriltag_configs/crescendo/crescendo_apriltags.json"):
    # add apritag
    config = spectacularAI.depthai.Configuration()
    # config.aprilTagPath = aprilTagPath
    config.useFeatureTracker = True
    config.useVioAutoExposure = True
    config.inputResolution = '800p'

    pipeline = depthai.Pipeline()
    vio_pipeline = spectacularAI.depthai.Pipeline(pipeline, config)

    RGB_OUTPUT_WIDTH = 800 # very small on purpose
    REF_ASPECT = 1920 / 1080.0
    w = RGB_OUTPUT_WIDTH
    h = int(round(w / REF_ASPECT))

    camRgb = pipeline.createColorCamera()
    camRgb.setPreviewSize(w, h)
    camRgb.setResolution(depthai.ColorCameraProperties.SensorResolution.THE_800_P)
    camRgb.setColorOrder(depthai.ColorCameraProperties.ColorOrder.BGR)
    camRgb.initialControl.setAutoFocusMode(depthai.RawCameraControl.AutoFocusMode.OFF)
    camRgb.initialControl.setManualFocus(0) # seems to be about 1m, not sure about the units
    camRgb.initialControl.setManualExposure(10000, 1000)
    out_source = camRgb.preview

    xout_camera = pipeline.createXLinkOut()
    xout_camera.setStreamName("rgb")
    out_source.link(xout_camera.input)

    return pipeline, vio_pipeline

class ApriltagLocalizer:
    def __init__(self):
        empty_xyz = lambda: { c: [] for c in 'xyz' }
        vio_data = empty_xyz()
        self.vio_data = vio_data

    def get_xyz_quat(self, pose: spectacularAI.Pose):
        position = pose.position
        xyz = position.x, position.y, position.z

        rotation = pose.orientation
        quat = rotation.w, rotation.x, rotation.y, rotation.z

        return xyz, quat
    
    def print_xyz_rot(self, pose: spectacularAI.Pose, ending="\n", digits=4):
        xyz, quat = self.get_xyz_quat(pose)

        xyz = tuple(round(x, digits) for x in xyz)
        quat = tuple(round(q, digits) for q in quat)

        print(f"xyz: {xyz}, quat: {quat}", end=ending)

    def start_in_parallel_with(self, parallel_thing):
        thread = threading.Thread(target = parallel_thing)
        thread.start()
        thread.join()

    @staticmethod
    def fudge_factor(x, y, yaw):
        '''
        Manual correction for when the camera is angled
        '''
        return x, (y - 0.04 * np.sin(yaw)), yaw

if __name__ == '__main__':
    # SLAM
    pipeline, vio_pipeline = make_pipelines()

    cLogger = CustomLogger(debug_to_stdout=PRINT_LOGS)

    NetworkTables.initialize(server='10.85.92.2')
    sd = NetworkTables.getTable('SmartDashboard')

    sd.putNumber('vision_x', 0)
    sd.putNumber('vision_y', 0)
    sd.putNumber('vision_z', 0)
    sd.putNumber('vision_roll', 0)
    sd.putNumber('vision_yaw', 0)
    sd.putNumber('vision_pitch', 0)
    sd.putBoolean('vision_active', True)

    # AprilTag injection
    camera_params = get_camera_params()
    detector = apriltag.Detector(
        families="tag36h11",
        nthreads=4,
    )

    COV = 0.0001 # orientation covariance
    POSITION_COV = np_to_list(COV * np.eye(3)) # position covariance

    tag_info = load_tag_config("apriltag_configs/crescendo/crescendo_apriltags.json")

    # need to get a pose from apriltag to start the vio
    cLogger.log_info("Waiting to see apriltag for first pose...")

    with depthai.Device(pipeline) as device, \
        vio_pipeline.startSession(device) as vio_session:
    
        rgbQueue = device.getOutputQueue(name="rgb", maxSize=4, blocking=False)

        while True:
            inFrame = rgbQueue.get()
            apriltag_timestamp = inFrame.getTimestampDevice().total_seconds()
            frame = cv2.cvtColor(inFrame.getCvFrame(), cv2.COLOR_BGR2GRAY)

            # detect apriltag
            detections = detector.detect(frame, estimate_tag_pose=True, camera_params=camera_params, tag_size=0.1651)

            if len(detections) > 0:
                camera_poses = []
                for detection in detections:
                    print(detection.tag_id)
                    id = detection.tag_id
                    camera_pose = pose_to_transform(detection.pose_R, detection.pose_t)
                    camera_poses.append(camera_pose)

                global_pose = get_global_pose([(id, pose) for id, pose in zip([detection.tag_id for detection in detections], camera_poses)], tag_info)
                if global_pose is not None:
                    cLogger.log_info(f"Acquired gobal pose: {global_pose}")
                    break

        # correct with the apriltag pose
        sai_pose = spectacularAI.Pose.fromMatrix(apriltag_timestamp, np_to_list(global_pose))
        vio_session.addAbsolutePose(sai_pose, POSITION_COV, COV)

        cLogger.log_info("Starting VIO...")

        timing_info = []
        counter = 0

        # will need this for NetworkTables to communicate if we are activated
        activated = False # upon first vio output, set to True
        active = False # if we were activated last loop, set to True

        while True:
            t1 = time.perf_counter()
            ### VIO ###
            if vio_session.hasOutput(): # if we have a new vio output
                vio_out = vio_session.getOutput() # get it
                activated = True # we have a vio output, so we are activated
                active = True # we were activated last loop

                # matrix
                matrix = vio_out.pose.asMatrix() # should be global pose now
                # matrix = global_pose if global_pose is not None else np.eye(4) # TODO: hacky, fix this
                # if counter % 2 == 0:
                x, y, z = matrix[0, 3], matrix[1, 3], matrix[2, 3]
                roll, pitch, yaw = homogenous_to_euler(matrix)
                # x, y, yaw = apriltag_slam.fudge_factor(vio_out.pose.position.x, vio_out.pose.position.y, yaw)
                sd.putNumber('vision_x', x)
                sd.putNumber('vision_y', y)
                sd.putNumber('vision_z', z)
                sd.putNumber('vision_roll', roll)
                sd.putNumber('vision_yaw', yaw)
                sd.putNumber('vision_pitch', pitch)
                sd.putBoolean('vision_active', True)
                roll_deg, pitch_deg, yaw_deg = [np.rad2deg(roll), np.rad2deg(pitch), np.rad2deg(yaw)]
                if counter % 2 == 0:
                    cLogger.log_debug(f"(x, y, z, yaw (deg)): {(x, y, z, yaw_deg)}")
                    
            elif rgbQueue.has(): # if we have a new rgb frame
                rgbFrame = rgbQueue.get()
                apriltag_timestamp = rgbFrame.getTimestampDevice().total_seconds()

                cvFrame = cv2.cvtColor(rgbFrame.getCvFrame(), cv2.COLOR_BGR2GRAY)

                # once every few frames, update w/ apriltag again
                if counter % 10 == 0:
                    # detect apriltag (only ever 2)
                    detections = detector.detect(cvFrame, estimate_tag_pose=True, camera_params=camera_params, tag_size=0.1651)[0:1]
                    if len(detections) > 0:
                        global_pose = get_global_pose([(id, pose_to_transform(detection.pose_R, detection.pose_t)) for detection in detections], tag_info)
                        if global_pose is not None:
                            sai_pose = spectacularAI.Pose.fromMatrix(apriltag_timestamp, np_to_list(global_pose))
                            vio_session.addAbsolutePose(sai_pose, POSITION_COV, COV)
                            cLogger.log_info(f"Global pose from Tag {detections[0].tag_id}: {global_pose[0][3]}, {global_pose[1][3]}, {global_pose[2][3]}")

                if SHOW_CAM:
                    if active and activated:
                        cv2.putText(cvFrame, "Active", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                    elif not active and activated:
                        cv2.putText(cvFrame, "No VIO", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
                    elif not activated and not active:
                        cv2.putText(cvFrame, "Not Active", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
                    cv2.imshow("rgb", cvFrame)
                    cv_key = cv2.waitKey(1)
                    if cv_key == ord('q'):
                        break
            else: # if we have neither a new vio output nor a new rgb frame
                active = False
                sd.putBoolean('vision_active', False)
                time.sleep(0.005) # need to sleep to not spinlock

            ### VIO ###
            dt = time.perf_counter() - t1
            timing_info.append(dt)
            counter += 1
            if counter % 1000 == 0 and activated:
                cLogger.log_info(f"Average speed (last 1000): {1/np.mean(timing_info)} Hz \n")
                timing_info.clear()
