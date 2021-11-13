import math
import cv2
import depthai as dai
from mpl_toolkits import mplot3d
import matplotlib.pyplot as plt
import numpy as np
import UVdisp
import obstacleDetect
import aStar

# Closer-in minimum depth, disparity range is doubled (from 95 to 190):
extended_disparity = False  # doesn't work
# Better accuracy for longer distance, fractional disparity 32-levels:
subpixel = True
# Better handling for occlusions:
lr_check = True

focalLen = 441.25*31.35
baseline = 7.5*10

# Create pipeline
pipeline = dai.Pipeline()

# Define sources and outputs
monoLeft = pipeline.createMonoCamera()
monoRight = pipeline.createMonoCamera()

depth = pipeline.createStereoDepth()

outDisp = pipeline.createXLinkOut()
outDisp.setStreamName("depOut")

outR = pipeline.createXLinkOut()
outR.setStreamName("outR")

# Properties
monoLeft.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
monoLeft.setBoardSocket(dai.CameraBoardSocket.LEFT)
monoRight.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
monoRight.setBoardSocket(dai.CameraBoardSocket.RIGHT)

# Create a node that will produce the depth map (using disparity output as it's easier to visualize depth this way)
depth.initialConfig.setConfidenceThreshold(200)
# Options: MEDIAN_OFF, KERNEL_3x3, KERNEL_5x5, KERNEL_7x7 (default)
depth.initialConfig.setMedianFilter(dai.MedianFilter.KERNEL_7x7)
depth.setLeftRightCheck(subpixel)
depth.setExtendedDisparity(extended_disparity)
depth.setSubpixel(lr_check)

# Linking
monoLeft.out.link(depth.left)
monoRight.out.link(depth.right)

monoRight.out.link(outR.input)

depth.depth.link(outDisp.input)

if __name__ == "__main__":
    # Connect to device and start pipeline
    with dai.Device(pipeline) as device:

        # Output queue will be used to get the disparity frames from the outputs defined above
        depQ = device.getOutputQueue(name="depOut", maxSize=4, blocking=False)
        rQ = device.getOutputQueue(name="outR", maxSize=4, blocking=False)

        plt.ion()
        plt.show()

        its = 0
        counter = 0

        while True:
            inDisp = depQ.get()  # blocking call, will wait until a new data has arrived
            inR = rQ.get()

            frameDep = inDisp.getFrame()
            frameR = inR.getCvFrame()

            dispCalc = np.divide(focalLen*baseline, frameDep)

            depCol = cv2.applyColorMap(cv2.convertScaleAbs(frameDep*10, alpha=(255.0/65535.0)), cv2.COLORMAP_TWILIGHT)
            cv2.imshow("depth", depCol)
            cv2.imshow("dispCalc", cv2.applyColorMap(cv2.convertScaleAbs(dispCalc*10, alpha=(255.0/65535.0)), cv2.COLORMAP_TWILIGHT))

            vDisp, uDisp = UVdisp.uvDisp(dispCalc)
            shellFlat, unknFlat, walkFlat = obstacleDetect.detect(vDisp, dispCalc, frameDep, verbose=False)

            onPath, path, closestNode, voro, walkmap = \
                aStar.aStar(shellFlat, unknFlat, walkFlat, (0, shellFlat.shape[1] - 1), verbose=False,
                            distFunc=aStar.euclid, goalFunc=aStar.euclid, voroFunc=aStar.euclid)

            # plt.figure(1)
            # plt.imshow(onPath.astype(np.uint8)*255/2 + walkmap.astype(np.uint8)*255/2)
            # plt.figure(2)
            plt.imshow(voro + onPath.astype(np.uint8)*5)
            # plt.show()
            plt.pause(0.1)

            its += 1
