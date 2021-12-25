import math
import cv2
import depthai as dai
from mpl_toolkits import mplot3d
import matplotlib.pyplot as plt
import numpy as np
import UVdisp
import obstacleDetect

# Closer-in minimum depth, disparity range is doubled (from 95 to 190):
extended_disparity = False  # doesn't work
# Better accuracy for longer distance, fractional disparity 32-levels:
subpixel = True
# Better handling for occlusions:
lr_check = True

imgid = 'ROT-' + ('SP' if subpixel else '') + ('LR' if lr_check else '')

focalLen = 441.25*31.35
baseline = 7.5*10
# focalLen = 2000

# Create pipeline
pipeline = dai.Pipeline()

# Define sources and outputs
monoLeft = pipeline.createMonoCamera()
monoRight = pipeline.createMonoCamera()

disp = pipeline.createStereoDepth()

outDisp = pipeline.createXLinkOut()
outDisp.setStreamName("dispOut")

outL = pipeline.createXLinkOut()
outL.setStreamName("outL")
outR = pipeline.createXLinkOut()
outR.setStreamName("outR")

# Properties
monoLeft.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
monoLeft.setBoardSocket(dai.CameraBoardSocket.LEFT)
monoRight.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
monoRight.setBoardSocket(dai.CameraBoardSocket.RIGHT)

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

disp.depth.link(outDisp.input)


def mapVal(inSt, inEn, outSt, outEn, val):
    return (val-inSt)/(inEn-inSt) * (outEn-outSt) + outSt


amRot = 3

if __name__ == "__main__":
    # Connect to device and start pipeline
    with dai.Device(pipeline) as device:

        # Output queue will be used to get the disparity frames from the outputs defined above
        q = device.getOutputQueue(name="dispOut", maxSize=4, blocking=False)
        qL = device.getOutputQueue(name="outL", maxSize=4, blocking=False)
        qR = device.getOutputQueue(name="outR", maxSize=4, blocking=False)

        while True:

            vDisps = []
            deps = []
            disps = []
            rots = []

            inDisp = q.get()  # blocking call, will wait until a new data has arrived

            inL = qL.get()
            inR = qR.get()

            for i in range(1, amRot+1):
                rot = mapVal(1, amRot, 45, 180-45, i)

                inDisp = q.get()  # blocking call, will wait until a new data has arrived

                inL = qL.get()
                inR = qR.get()

                frame = inDisp.getFrame()

                frameL = inL.getCvFrame()
                frameR = inR.getCvFrame()

                # Normalization for better visualization
                frameDep = cv2.convertScaleAbs(frame*10, alpha=(255.0/65535.0))

                frameDispCalc = np.divide(focalLen*baseline, frame)

                # frameCol = cv2.cvtColor(frameCol, cv2.COLOR_BGR2RGB)

                # Available color maps: https://docs.opencv.org/3.4/d3/d50/group__imgproc__colormap.html
                frameDep = cv2.applyColorMap(frameDep, cv2.COLORMAP_TWILIGHT)
                vDisp, uDisp = UVdisp.uvDisp(frameDispCalc.astype(np.uint16))
                cv2.imshow("depth", frameDep)
                cv2.imshow("dispCalc", cv2.applyColorMap(cv2.convertScaleAbs(frameDispCalc*10, alpha=(255.0/65535.0)), cv2.COLORMAP_TWILIGHT))
                cv2.imshow("v", vDisp*10)

                vDisps.append(vDisp)
                disps.append(frameDispCalc)
                deps.append(frame)
                rots.append(np.deg2rad(rot))

                if cv2.waitKey() == ord('q'):
                    break

            obstacleDetect.detectMult(vDisps, disps, deps, rots, True, True)
