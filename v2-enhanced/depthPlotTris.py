import math
import cv2
import depthai as dai
from mpl_toolkits import mplot3d
from mpl_toolkits.mplot3d import Axes3D
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
maxVal = 5000
minVal = 100

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

# axTmp = Axes3D(fig)
# axTmp.set_xlim3d(-1000, 1000)
# axTmp.set_ylim3d(-1000, 1000)
# axTmp.set_zlim3d(-1000, 1000)

# ax.axis('equal')

ax.view_init(98, 6)

# ax.set_title('surface')

surf = None


def mapPoints(inX, inY, dep):
    dep = -dep

    # hypLen = math.sqrt(inX**2 + inY**2 + focalLen**2)
    conv = dep/focalLen
    return inX * conv, inY * conv, focalLen * conv


def plotDisp(xyz, tris):
    global surf

    print(xyz.shape)

    X = xyz[:, 0]
    Y = xyz[:, 1]
    Z = xyz[:, 2]

    # surf = ax.scatter(xs=X, ys=Y, zs=Z, color='green')

    if surf is None:
        # surf = ax.plot_surface(X, Y, Z, rstride=1, cstride=1, cmap='viridis', edgecolor='none')
        # surf = ax.plot(xs=X, ys=Y, zs=Z, color='green', marker='o', markersize=0.1, linewidth=0)[0]
        # surf = ax.scatter(X, Y, Z, c=Z, cmap='viridis', linewidth=0.5)
        surf = ax.plot_trisurf(X, Y, Z, triangles=triangulation, cmap='viridis', linewidths=0.2)
    else:
        # surf.set_xdata(X)
        # surf.set_ydata(Y)
        # surf.set_zdata(Z)
        # surf.set_data_3d(X, Y, Z)
        surf = ax.plot_trisurf(X, Y, Z, triangles=triangulation, cmap='viridis', linewidths=0.2)
        # surf = ax.plot(xs=X, ys=Y, zs=Z, color='green', marker='o', markersize=0.2, linewidth=0)

    plt.draw()


def filt(inn):
    return minVal < inn < maxVal


if __name__ == "__main__":
    # Connect to device and start pipeline
    with dai.Device(pipeline) as device:

        # Output queue will be used to get the disparity frames from the outputs defined above
        q = device.getOutputQueue(name="depthOut", maxSize=4, blocking=False)

        frames = []

        plt.ion()
        plt.show()

        its = 0

        while True:
            inDepth = q.get()  # blocking call, will wait until a new data has arrived
            frame = inDepth.getFrame()

            if len(frames) >= 1:
                frames.pop(0)

            frames.append(frame)

            frameAVG = 0

            for i in frames:
                frameAVG = frameAVG + i

            frameAVG = frameAVG / len(frames)

            # blank_image = np.zeros((100, 100), np.uint16)
            # blank_image[:, :] = 2000

            xyzRaw = []
            triangulation = []

            rows, cols = frame.shape

            blurred = cv2.GaussianBlur(frame, (11, 11), cv2.BORDER_DEFAULT)
            resized = cv2.resize(blurred, (int(cols/10), int(rows/10)), interpolation=cv2.INTER_AREA)

            final = resized

            rows, cols = final.shape
            for i in range(rows):
                for j in range(cols):
                    k = final[i, j]

                    if i > 0 and j > 0 and filt(final[i-1][j]) and filt(final[i][j-1]) and filt(final[i-1][j-1]):
                        triangulation.append(((i-1)*cols+(j), (i)*cols+(j-1), (i-1)*cols+(j-1)))
                    if i < rows-1 and j < cols-1 and filt(final[i+1][j]) and filt(final[i][j+1]) and filt(final[i+1][j+1]):
                        triangulation.append(((i+1)*cols+(j), (i)*cols+(j+1), (i+1)*cols+(j+1)))

                    if filt(k):
                        tmp = mapPoints(i - rows / 2, j - cols / 2, k)
                        xyzRaw.append(tmp)
                    else:
                        xyzRaw.append((0, 0, 0))

            xyz = np.array(xyzRaw)

            # Normalization for better visualization
            frameDisp = cv2.convertScaleAbs(final*10, alpha=(255.0 / 65535.0))

            # Available color maps: https://docs.opencv.org/3.4/d3/d50/group__imgproc__colormap.html
            frameDisp = cv2.applyColorMap(frameDisp, cv2.COLORMAP_TWILIGHT)
            cv2.imshow("disparity off", frameDisp)

            if its >= 5:
                plotDisp(xyz, triangulation)
                ax.set_xlim(-2500, 2500)
                ax.set_ylim(-2500, 2500)
                ax.set_zlim(60000, 65000)
                # plt.show()
                cv2.waitKey()

            if cv2.waitKey(1) == ord('q'):
                break

            its += 1
