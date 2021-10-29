import hough
import matplotlib.pyplot as plt
import cv2
import numpy as np

focalLen = 19.6 * 10
baseline = 7.5 * 10


fig = plt.figure()
ax = plt.axes()


def mapPoints(inX, inY, dispAm):
    if dispAm == 0:
        return None

    dep = focalLen * baseline / dispAm

    # hypLen = math.sqrt(inX**2 + inY**2 + focalLen**2)
    conv = dep/focalLen
    return inX * conv, inY * conv, focalLen * conv


if __name__ == "__main__":

    for i in range(10):
        # Constructing test image
        imgId = "SPLR-" + str(i) + ".png"
        vDisp = cv2.imread("vDisp/v" + imgId, cv2.IMREAD_UNCHANGED)
        disp = cv2.imread("dispImages/depth" + imgId, cv2.IMREAD_UNCHANGED)
        col = cv2.imread("RImages/R" + imgId, cv2.IMREAD_COLOR)

        _, dispSLIC, dispScaled, floor = hough.hough(disp, vDisp, slicc=False, slicRef=col, verbose=True, m=40, k=10, scl=2)

        dispFloorless = dispSLIC * floor

        obs = np.zeros(dispFloorless.shape[1], dtype=np.float_)

        rows, cols = dispFloorless.shape
        for row in range(rows):
            for col in range(cols):
                coords = mapPoints(row, col, dispFloorless[row, col])
                if coords is not None:
                    obs[col] = max(obs[col], coords[2])

        # print(obs)
        cv2.imshow("floorless", cv2.convertScaleAbs(dispFloorless*10, alpha=(255.0/65535.0)))
        cv2.imshow("slic", cv2.convertScaleAbs(dispSLIC*10, alpha=(255.0/65535.0)))
        cv2.imshow("scaled", dispScaled)
        cv2.imshow("floor", floor*255)

        ax.plot(range(cols), -obs, "ko", markersize=0.5)

        plt.show()

