import numpy as np
import cv2
import math
import scipy
from scipy import signal
from scipy import ndimage

m = 20


def SLICdist(image, aCoords, bCoords, s):
    global m

    aPlace = image[aCoords[0], aCoords[1]]
    bPlace = image[bCoords[0], bCoords[1]]

    dc = math.sqrt((aPlace[0]-bPlace[0])**2 + (aPlace[1]-bPlace[1])**2 + (aPlace[2]-bPlace[2])**2)
    ds = math.sqrt((aCoords[0]-bCoords[0])**2 + (aCoords[1]-bCoords[1])**2)

    return math.sqrt(dc**2 + (ds/s)**2 * m**2)


def SLIC(image, k, prevLab=None, prevDist=None):
    s = int(math.sqrt(image.shape[0]*image.shape[1]/k))
    c = []

    kernely = np.array([[1, 1, 1], [0, 0, 0], [-1, -1, -1]])
    kernelx = np.array([[1, 0, -1], [1, 0, -1], [1, 0, -1]])

    gradX = cv2.filter2D(image, cv2.CV_8U, kernelx)
    gradY = cv2.filter2D(image, cv2.CV_8U, kernely)

    gradRGB = np.sqrt(gradY**2 + gradX**2)

    grad = np.sqrt(gradRGB[..., 0]**2 + gradRGB[..., 1]**2 + gradRGB[..., 2]**2)

    for row in range(1, int(image.shape[0]/s)):
        for col in range(1, int(image.shape[1]/s)):
            c.append([row*s, col*s])

    for cc in c:
        tmpRow = cc[0]
        tmpCol = cc[1]
        for row in range(tmpRow-1, tmpRow+2):
            for col in range(tmpCol-1, tmpCol+2):
                if grad[cc[0], cc[1]] > grad[row, col]:
                    cc[0] = row
                    cc[1] = col

    # pixLab = None
    # pixDist = None

    pixLab = np.ones((image.shape[0], image.shape[1]), dtype=np.uint8) * -1
    pixDist = np.ones((image.shape[0], image.shape[1]), dtype=np.float_) * np.inf

    # while True:
    for itt in range(100):

        print("it" + str(itt) + " starting")

        for ind in range(len(c)):
            cc = c[ind]
            tmpRow = cc[0]
            tmpCol = cc[1]
            for row in range(max(tmpRow - s, 0), min(tmpRow + s + 1, image.shape[0])):
                for col in range(max(tmpCol - s, 0), min(tmpCol + s + 1, image.shape[1])):
                    tmpDist = SLICdist(image, cc, [row, col], s)
                    if tmpDist < pixDist[row, col]:
                        pixDist[row, col] = tmpDist
                        pixLab[row, col] = ind

        newC = [[0, 0] for _ in range(len(c))]
        newCAm = [0 for _ in range(len(c))]
        cAvg = [[0, 0, 0] for _ in range(len(c))]

        for row in range(image.shape[0]):
            for col in range(image.shape[1]):
                tmp = newC[pixLab[row, col]]
                tmp[0] += row
                tmp[1] += col

                tmp = cAvg[pixLab[row, col]]
                tmp[0] += image[row, col, 0]
                tmp[1] += image[row, col, 1]
                tmp[2] += image[row, col, 2]

                newCAm[pixLab[row, col]] += 1

        sub = 0

        for ind in range(len(c)):
            indd = ind - sub

            if newCAm[indd] <= 0:
                newC.pop(indd)
                newCAm.pop(indd)
                sub += 1
                continue

            newC[indd][0] = int(newC[indd][0] / newCAm[indd])
            newC[indd][1] = int(newC[indd][1] / newCAm[indd])

            cAvg[indd][0] = int(cAvg[indd][0] / newCAm[indd])
            cAvg[indd][1] = int(cAvg[indd][1] / newCAm[indd])
            cAvg[indd][2] = int(cAvg[indd][2] / newCAm[indd])

        pixAvg = np.zeros(image.shape, dtype=np.uint8)

        for row in range(image.shape[0]):
            for col in range(image.shape[1]):
                pixAvg[row, col] = cAvg[pixLab[row, col]]

        c = newC

        print("it" + str(itt) + " complete")

        pixLabDisp = cv2.applyColorMap(pixLab.clip(min=0).astype(np.uint8), cv2.COLORMAP_TWILIGHT)

        # print(pixLab)
        print(np.max(pixLab))
        # print(pixLabDisp)
        # print(c)

        cv2.imshow("col", image)
        cv2.imshow("labs", pixLabDisp)
        cv2.imshow("avgs", pixAvg)

        cv2.waitKey(1)
        if itt == 98 or itt == 0:
            cv2.waitKey()

    return pixLab, pixDist


if __name__ == "__main__":
    for i in range(10):
        # Constructing test image
        imgId = "SPLR-" + str(i) + ".png"
        colour = cv2.imread("colImages/col" + imgId, cv2.IMREAD_UNCHANGED)
        disp = cv2.imread("dispImages/disp" + imgId, cv2.IMREAD_UNCHANGED)
        disp255 = cv2.convertScaleAbs(disp*10, alpha=(255.0/65535.0))

        colSmall = cv2.resize(colour, (int(disp.shape[1]/2), int(disp.shape[0]/2)), interpolation=cv2.INTER_AREA)

        labels, dists = SLIC(colSmall, 300)

