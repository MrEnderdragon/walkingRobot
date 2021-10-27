import numpy as np
import cv2
import math


def SLICdist(image, aCoords, bCoords, s, m):
    aPlace = image[aCoords[0], aCoords[1]]
    bPlace = image[bCoords[0], bCoords[1]]

    dc = math.sqrt((aPlace[0]-bPlace[0])**2 + (aPlace[1]-bPlace[1])**2 + (aPlace[2]-bPlace[2])**2)
    ds = math.sqrt((aCoords[0]-bCoords[0])**2 + (aCoords[1]-bCoords[1])**2)

    return math.sqrt(dc**2 + (ds/s)**2 * m**2)


def SLIC(image, k, m, its):
    s = int(math.sqrt(image.shape[0]*image.shape[1]/k))
    c = []

    kernely = np.array([[1, 1, 1], [0, 0, 0], [-1, -1, -1]])
    kernelx = np.array([[1, 0, -1], [1, 0, -1], [1, 0, -1]])

    gradX = cv2.filter2D(image, cv2.CV_8U, kernelx)
    gradY = cv2.filter2D(image, cv2.CV_8U, kernely)

    gradRGB = np.sqrt(gradY**2 + gradX**2)

    grad = np.sqrt(gradRGB[..., 0]**2 + gradRGB[..., 1]**2 + gradRGB[..., 2]**2)

    for row in range(1, int(image.shape[0]/s)+1):
        for col in range(1, int(image.shape[1]/s)+1):
            cur = [min(row*s, image.shape[0]-2), min(col*s, image.shape[1]-2)]
            tmpRow = cur[0]
            tmpCol = cur[1]
            for roww in range(tmpRow - 1, tmpRow + 2):
                for coll in range(tmpCol - 1, tmpCol + 2):
                    if grad[cur[0], cur[1]] > grad[roww, coll]:
                        cur[0] = roww
                        cur[1] = coll
            c.append(cur)

    pixLab = np.ones((image.shape[0], image.shape[1]), dtype=np.uint8) * -1
    pixDist = np.ones((image.shape[0], image.shape[1]), dtype=np.float_) * np.inf

    for itt in range(its):

        print("slic it" + str(itt) + " starting")

        for ind in range(len(c)):
            cc = c[ind]
            tmpRow = cc[0]
            tmpCol = cc[1]
            for row in range(max(tmpRow - s, 0), min(tmpRow + s + 1, image.shape[0])):
                for col in range(max(tmpCol - s, 0), min(tmpCol + s + 1, image.shape[1])):
                    tmpDist = SLICdist(image, cc, [row, col], s, m)
                    if tmpDist < pixDist[row, col]:
                        pixDist[row, col] = tmpDist
                        pixLab[row, col] = ind

        newC = [[0, 0] for _ in range(len(c))]
        newCAm = [0 for _ in range(len(c))]

        for row in range(image.shape[0]):
            for col in range(image.shape[1]):
                tmp = newC[pixLab[row, col]]
                tmp[0] += row
                tmp[1] += col

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

        c = newC

        print("slic it" + str(itt) + " finished")

    return pixLab, pixDist, c
