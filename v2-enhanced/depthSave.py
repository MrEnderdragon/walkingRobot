import math
import cv2
import depthai as dai
from mpl_toolkits import mplot3d
import matplotlib.pyplot as plt
import numpy as np

# Closer-in minimum depth, disparity range is doubled (from 95 to 190):
extended_disparity = False  # doesn't work
# Better accuracy for longer distance, fractional disparity 32-levels:
subpixel = True
# Better handling for occlusions:
lr_check = True

focalLen = 19.6 * 100
# focalLen = 2000

# Create pipeline
pipeline = dai.Pipeline()

# Define sources and outputs
monoLeft = pipeline.createMonoCamera()
monoRight = pipeline.createMonoCamera()
depth = pipeline.createStereoDepth()

out = pipeline.createXLinkOut()
out.setStreamName("depthOut")

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

depth.depth.link(out.input)

fig = plt.figure()
ax = plt.axes(projection='3d')
ax.set_xlabel('x')
ax.set_ylabel('y')
ax.set_zlabel('z')

# ax.axes.set_xlim3d(left=-100, right=100)
# ax.axes.set_ylim3d(bottom=-100, top=100)
# ax.axes.set_zlim3d(bottom=-100, top=100)

# ax.set_title('surface')

surf = None


def mapPoints(inX, inY, dep):
    dep = -dep

    hypLen = math.sqrt(inX**2 + inY**2 + focalLen**2)
    conv = dep/hypLen
    return inX * conv, inY * conv, focalLen * conv


def plotDisp(xyz):
    global surf

    print(xyz.shape)

    X = xyz[:, 0]
    Y = xyz[:, 1]
    Z = xyz[:, 2]

    # surf = ax.scatter(xs=X, ys=Y, zs=Z, color='green')

    if surf is None:
        # surf = ax.plot_surface(X, Y, Z, rstride=1, cstride=1, cmap='viridis', edgecolor='none')
        surf = ax.plot(xs=X, ys=Y, zs=Z, color='green', marker='o', markersize=0.1, linewidth=0)[0]
        # surf = ax.scatter(X, Y, Z, c=Z, cmap='viridis', linewidth=0.5)
    else:
        # surf.set_xdata(X)
        # surf.set_ydata(Y)
        # surf.set_zdata(Z)
        surf.set_data_3d(X, Y, Z)
        # surf = ax.plot(xs=X, ys=Y, zs=Z, color='green', marker='o', markersize=0.2, linewidth=0)

    plt.draw()


if __name__ == "__main__":
    # Connect to device and start pipeline
    with dai.Device(pipeline) as device:

        # Output queue will be used to get the disparity frames from the outputs defined above
        q = device.getOutputQueue(name="depthOut", maxSize=4, blocking=False)

        frames = []

        its = 0

        counter = 0

        while True:
            inDepth = q.get()  # blocking call, will wait until a new data has arrived
            frame = inDepth.getFrame()

            if len(frames) >= 1:
                frames.pop(0)

            frames.append(frame)

            # Normalization for better visualization
            frameDisp = cv2.convertScaleAbs(frame*10, alpha=(255.0/65535.0))

            # Available color maps: https://docs.opencv.org/3.4/d3/d50/group__imgproc__colormap.html
            frameDisp = cv2.applyColorMap(frameDisp, cv2.COLORMAP_TWILIGHT)
            cv2.imshow("disparity off", frameDisp)

            if cv2.waitKey() == ord('s'):
                cv2.imwrite('./depthImagesOLD/depthSPLR-' + str(counter) + '.png', frame.astype(np.uint16))
                counter += 1

            if cv2.waitKey(1) == ord('q'):
                break

            its += 1
