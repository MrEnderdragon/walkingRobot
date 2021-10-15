import cv2
import depthai as dai
import numpy as np

# Closer-in minimum depth, disparity range is doubled (from 95 to 190):
extended_disparity = False
# Better accuracy for longer distance, fractional disparity 32-levels:
subpixel = True
# Better handling for occlusions:
lr_check = True

# Create pipeline
pipeline = dai.Pipeline()

# Define sources and outputs
monoLeft = pipeline.createMonoCamera()
monoRight = pipeline.createMonoCamera()
depth7 = pipeline.createStereoDepth()
depth5 = pipeline.createStereoDepth()
depth3 = pipeline.createStereoDepth()
depthoff = pipeline.createStereoDepth()

out7 = pipeline.createXLinkOut()
out5 = pipeline.createXLinkOut()
out3 = pipeline.createXLinkOut()
outoff = pipeline.createXLinkOut()

out7.setStreamName("disparity7")
out5.setStreamName("disparity5")
out3.setStreamName("disparity3")
outoff.setStreamName("disparityoff")

# Properties
monoLeft.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
monoLeft.setBoardSocket(dai.CameraBoardSocket.LEFT)
monoRight.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
monoRight.setBoardSocket(dai.CameraBoardSocket.RIGHT)

# Create a node that will produce the depth map (using disparity output as it's easier to visualize depth this way)
depth7.initialConfig.setConfidenceThreshold(200)
# Options: MEDIAN_OFF, KERNEL_3x3, KERNEL_5x5, KERNEL_7x7 (default)
depth7.initialConfig.setMedianFilter(dai.MedianFilter.KERNEL_7x7)
depth7.setLeftRightCheck(lr_check)
depth7.setExtendedDisparity(extended_disparity)
depth7.setSubpixel(subpixel)

# Create a node that will produce the depth map (using disparity output as it's easier to visualize depth this way)
depth5.initialConfig.setConfidenceThreshold(200)
# Options: MEDIAN_OFF, KERNEL_3x3, KERNEL_5x5, KERNEL_7x7 (default)
depth5.initialConfig.setMedianFilter(dai.MedianFilter.KERNEL_7x7)
depth5.setLeftRightCheck(lr_check)
depth5.setExtendedDisparity(extended_disparity)
depth5.setSubpixel(False)

# Create a node that will produce the depth map (using disparity output as it's easier to visualize depth this way)
depth3.initialConfig.setConfidenceThreshold(200)
# Options: MEDIAN_OFF, KERNEL_3x3, KERNEL_5x5, KERNEL_7x7 (default)
depth3.initialConfig.setMedianFilter(dai.MedianFilter.KERNEL_7x7)
depth3.setLeftRightCheck(False)
depth3.setExtendedDisparity(extended_disparity)
depth3.setSubpixel(subpixel)

# Create a node that will produce the depth map (using disparity output as it's easier to visualize depth this way)
depthoff.initialConfig.setConfidenceThreshold(200)
# Options: MEDIAN_OFF, KERNEL_3x3, KERNEL_5x5, KERNEL_7x7 (default)
depthoff.initialConfig.setMedianFilter(dai.MedianFilter.KERNEL_7x7)
depthoff.setLeftRightCheck(False)
depthoff.setExtendedDisparity(extended_disparity)
depthoff.setSubpixel(False)

# Linking
monoLeft.out.link(depth7.left)
monoRight.out.link(depth7.right)

monoLeft.out.link(depth5.left)
monoRight.out.link(depth5.right)

monoLeft.out.link(depth3.left)
monoRight.out.link(depth3.right)

monoLeft.out.link(depthoff.left)
monoRight.out.link(depthoff.right)

depth7.disparity.link(out7.input)
depth5.disparity.link(out5.input)
depth3.disparity.link(out3.input)
depthoff.disparity.link(outoff.input)


if __name__ == "__main__":
    # Connect to device and start pipeline
    with dai.Device(pipeline) as device:

        # Output queue will be used to get the disparity frames from the outputs defined above
        q7 = device.getOutputQueue(name="disparity7", maxSize=4, blocking=False)
        q5 = device.getOutputQueue(name="disparity5", maxSize=4, blocking=False)
        q3 = device.getOutputQueue(name="disparity3", maxSize=4, blocking=False)
        qoff = device.getOutputQueue(name="disparityoff", maxSize=4, blocking=False)

        while True:
            inDepth7 = q7.get()  # blocking call, will wait until a new data has arrived
            inDepth5 = q5.get()  # blocking call, will wait until a new data has arrived
            inDepth3 = q3.get()  # blocking call, will wait until a new data has arrived
            inDepthoff = qoff.get()  # blocking call, will wait until a new data has arrived

            frame7 = inDepth7.getFrame()
            frame5 = inDepth5.getFrame()
            frame3 = inDepth3.getFrame()
            frameoff = inDepthoff.getFrame()

            # Normalization for better visualization
            frame7 = (frame7 * (255 / depth7.getMaxDisparity())).astype(np.uint8)
            frame5 = (frame5 * (255 / depth5.getMaxDisparity())).astype(np.uint8)
            frame3 = (frame3 * (255 / depth3.getMaxDisparity())).astype(np.uint8)
            frameoff = (frameoff * (255 / depthoff.getMaxDisparity())).astype(np.uint8)


            # Available color maps: https://docs.opencv.org/3.4/d3/d50/group__imgproc__colormap.html
            frame7 = cv2.applyColorMap(frame7, cv2.COLORMAP_TWILIGHT)
            frame5 = cv2.applyColorMap(frame5, cv2.COLORMAP_TWILIGHT)
            frame3 = cv2.applyColorMap(frame3, cv2.COLORMAP_TWILIGHT)
            frameoff = cv2.applyColorMap(frameoff, cv2.COLORMAP_TWILIGHT)
            cv2.imshow("disparity LRS", frame7)
            cv2.imshow("disparity LR", frame5)
            cv2.imshow("disparity S", frame3)
            cv2.imshow("disparity off", frameoff)

            if cv2.waitKey(1) == ord('q'):
                break