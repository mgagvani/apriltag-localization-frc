"""
AprilTag SLAM using DepthAI and SpectacularAI.
Includes 3D visualization using matplotlib and live camera preview. 
Requirements: pip install matplotlib opencv-python
"""
import depthai
import time
import matplotlib.pyplot as plt
import spectacularAI
import threading
import numpy as np

from quaternion2theta import quaternion_to_theta

SHOW_CAM = True
if SHOW_CAM:
    import cv2

def make_pipelines(aprilTagPath="apriltag_configs/test_apriltags.json"):
    # add apritag
    config = spectacularAI.depthai.Configuration()
    config.aprilTagPath = aprilTagPath

    pipeline = depthai.Pipeline()
    vio_pipeline = spectacularAI.depthai.Pipeline(pipeline, config)

    RGB_OUTPUT_WIDTH = 200 # very small on purpose
    REF_ASPECT = 1920 / 1080.0
    w = RGB_OUTPUT_WIDTH
    h = int(round(w / REF_ASPECT))

    camRgb = pipeline.createColorCamera()
    camRgb.setPreviewSize(w, h)
    camRgb.setResolution(depthai.ColorCameraProperties.SensorResolution.THE_1080_P)
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
    
    def print_vio(self, vio_out):
        '''
        - don't use (kept it for reference of `asMatrix())`
        '''
        view_mat = vio_out.pose.asMatrix() # 4x4 matrix (dont need this)
        return DeprecationWarning

    def start_in_parallel_with(self, parallel_thing):
        thread = threading.Thread(target = parallel_thing)
        thread.start()
        plt.show()
        thread.join()

if __name__ == '__main__':
    pipeline, vio_pipeline = make_pipelines()

    with depthai.Device(pipeline) as device, \
        vio_pipeline.startSession(device) as vio_session:

        apriltag_slam = ApriltagLocalizer()

        def main_loop():
            rgbQueue = device.getOutputQueue(name="rgb", maxSize=4, blocking=False)

            timing_info = []
            counter = 0

            while True:
                t1 = time.perf_counter()
                ### VIO ###
                if vio_session.hasOutput(): # if we have a new vio output
                    vio_out = vio_session.getOutput() # get it

                    apriltag_slam.print_xyz_rot(vio_out.pose, ending="\r")
                    # print(np.rad2deg(quaternion_to_theta(apriltag_slam.get_xyz_quat(vio_out.pose)[1])), end="\r")

                elif rgbQueue.has(): # if we have a new rgb frame
                    rgbFrame = rgbQueue.get()

                    if SHOW_CAM: # debug viewer (so we know if apriltag in frame)
                        cv2.imshow("rgb", rgbFrame.getCvFrame())

                        cv_key = cv2.waitKey(1)
                        if cv_key == ord('q'):
                            break
                else: # if we have neither a new vio output nor a new rgb frame
                    time.sleep(0.005) # need to sleep to not race condition

                ### VIO ###
                dt = time.perf_counter() - t1
                timing_info.append(dt)
                counter += 1
                if counter % 1000 == 0:
                    print(f"Average speed (last 1000): {1/np.mean(timing_info)} Hz \n")
                    timing_info.clear()


        apriltag_slam.start_in_parallel_with(main_loop)
