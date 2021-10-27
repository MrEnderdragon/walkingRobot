import math
import cv2
import depthai as dai
from mpl_toolkits import mplot3d
import matplotlib.pyplot as plt
import numpy as np

# Closer-in minimum depth, disparity range is doubled (from 95 to 190):
extended_disparity = False  # doesn't work
# Better accuracy for longer distance, fractional disparity 32-levels:
subpixel = False
# Better handling for occlusions:
lr_check = False

focalLen = 19.6 * 100
# focalLen = 2000

# Create pipeline
pipeline = dai.Pipeline()

# Define sources and outputs
monoLeft = pipeline.createMonoCamera()
monoRight = pipeline.createMonoCamera()

col = pipeline.createColorCamera()
stillEncoder = pipeline.createVideoEncoder()
stillEncoder.setDefaultProfilePreset(col.getStillSize(), 1, dai.VideoEncoderProperties.Profile.MJPEG)

disp = pipeline.createStereoDepth()

outDisp = pipeline.createXLinkOut()
outDisp.setStreamName("dispOut")

outCol = pipeline.createXLinkOut()
outCol.setStreamName("colOut")

outL = pipeline.createXLinkOut()
outL.setStreamName("outL")
outR = pipeline.createXLinkOut()
outR.setStreamName("outR")

controlIn = pipeline.createXLinkIn()
controlIn.setStreamName('control')

# Properties
monoLeft.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
monoLeft.setBoardSocket(dai.CameraBoardSocket.LEFT)
monoRight.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
monoRight.setBoardSocket(dai.CameraBoardSocket.RIGHT)

col.setBoardSocket(dai.CameraBoardSocket.RGB)
col.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
col.setInterleaved(False)
col.setColorOrder(dai.ColorCameraProperties.ColorOrder.RGB)

# Create a node that will produce the depth map (using disparity output as it's easier to visualize depth this way)
disp.initialConfig.setConfidenceThreshold(200)
# Options: MEDIAN_OFF, KERNEL_3x3, KERNEL_5x5, KERNEL_7x7 (default)
disp.initialConfig.setMedianFilter(dai.MedianFilter.KERNEL_7x7)
disp.setLeftRightCheck(subpixel)
disp.setExtendedDisparity(extended_disparity)
disp.setSubpixel(lr_check)

# Linking
monoLeft.out.link(disp.left)
monoRight.out.link(disp.right)

monoLeft.out.link(outL.input)
monoRight.out.link(outR.input)

disp.disparity.link(outDisp.input)
col.still.link(stillEncoder.input)
controlIn.out.link(col.inputControl)
stillEncoder.bitstream.link(outCol.input)

if __name__ == "__main__":
    # Connect to device and start pipeline
    with dai.Device(pipeline) as device:

        # Output queue will be used to get the disparity frames from the outputs defined above
        q = device.getOutputQueue(name="dispOut", maxSize=4, blocking=False)
        qCol = device.getOutputQueue(name="colOut", maxSize=4, blocking=False)
        qL = device.getOutputQueue(name="outL", maxSize=4, blocking=False)
        qR = device.getOutputQueue(name="outR", maxSize=4, blocking=False)

        controlQueue = device.getInputQueue('control')

        its = 0
        counter = 0

        while True:

            ctrl = dai.CameraControl()
            ctrl.setCaptureStill(True)
            controlQueue.send(ctrl)

            inDepth = q.get()  # blocking call, will wait until a new data has arrived
            inCol = qCol.tryGetAll()

            inL = qL.get()
            inR = qR.get()

            frame = inDepth.getFrame()

            frameCol = frame

            if len(inCol) > 0:
                frameCol = cv2.imdecode(inCol[0].getData(), cv2.IMREAD_UNCHANGED)

            frameL = inL.getCvFrame()
            frameR = inR.getCvFrame()

            # Normalization for better visualization
            frameDisp = cv2.convertScaleAbs(frame*10, alpha=(255.0/65535.0))

            # frameCol = cv2.cvtColor(frameCol, cv2.COLOR_BGR2RGB)

            # Available color maps: https://docs.opencv.org/3.4/d3/d50/group__imgproc__colormap.html
            frameDisp = cv2.applyColorMap(frameDisp, cv2.COLORMAP_TWILIGHT)
            cv2.imshow("disparity", frameDisp)
            cv2.imshow("color", frameCol)

            if cv2.waitKey() == ord('s'):
                cv2.imwrite('./dispImages/depth' + ('SP' if subpixel else '') + ('LR' if lr_check else '') + '-' + str(counter) + '.png', frame.astype(np.uint16))
                cv2.imwrite('./colImages/col' + ('SP' if subpixel else '') + ('LR' if lr_check else '') + '-' + str(counter) + '.png', frameCol)
                cv2.imwrite('./LImages/L' + ('SP' if subpixel else '') + ('LR' if lr_check else '') + '-' + str(counter) + '.png', frameL)
                cv2.imwrite('./RImages/R' + ('SP' if subpixel else '') + ('LR' if lr_check else '') + '-' + str(counter) + '.png', frameR)
                counter += 1

            if cv2.waitKey(1) == ord('q'):
                break

            its += 1
