import numpy as np
import cv2
from skimage.transform import hough_line, hough_line_peaks
import slic
import math


def hough(disp, vDisp, slicc=False, slicRef=None, verbose=False, **args):

    """
    :param disp: disparity image
    :param vDisp: v-disparity image
    :param slicc: run slic?
    :param slicRef: slic reference image
    :param verbose: print status?
    :param args: 'SLIC args: k, m, its, scl; HOUGH args: vertLimit, floorThresh'
    :return: (bestAng, bestDist), disparity image, floor mask
    """

    disp = np.copy(disp)

    # Constructing test image

    if slicc:
        if verbose:
            print("slic start")

        refSmall = cv2.resize(slicRef, (int(disp.shape[1] / (args["scl"] if "scl" in args else 1)), int(disp.shape[0] / (args["scl"] if "scl" in args else 1))), interpolation=cv2.INTER_AREA)
        # print(refSmall.shape)
        pixLab, _, cc = slic.SLIC(refSmall, args["k"] if "k" in args else 100, args["m"] if "m" in args else 25, args["its"] if "its" in args else 5)

        avgs = np.zeros(len(cc), dtype=np.float_)
        amsRaw = np.zeros(len(cc))
        ams = np.zeros(len(cc))

        pixLab = cv2.resize(pixLab, (disp.shape[1], disp.shape[0]), interpolation=cv2.INTER_AREA)

        cv2.imshow("mm", cv2.applyColorMap(pixLab.astype(np.uint8)*10, cv2.COLORMAP_JET))

        rows, cols = disp.shape
        for row in range(rows):
            for col in range(cols):
                amsRaw[pixLab[row, col]] += 1
                if disp[row, col] != 0:
                    avgs[pixLab[row, col]] += disp[row, col]
                    ams[pixLab[row, col]] += 1

        avgs = np.divide(avgs, ams, out=np.zeros_like(avgs), where=ams != 0)
        avgs[np.divide(ams, amsRaw, where=amsRaw != 0) < 0.1] = 0

        avgs = avgs.astype(np.uint32)

        for row in range(rows):
            for col in range(cols):
                if disp[row, col] == 0:
                    disp[row, col] = avgs[pixLab[row, col]]

    dispScaled = cv2.convertScaleAbs(disp*10, alpha=(255.0/65535.0))

    if verbose:
        print("hough1 start")

    # Classic straight-line Hough transform
    # Set a precision of 0.5 degree.
    tested_angles = np.linspace(-np.pi / 4, np.pi / 4, 360, endpoint=False)
    h, theta, d = hough_line(vDisp, theta=tested_angles)

    for _, angle, dist in zip(*hough_line_peaks(h, theta, d)):
        if -(args["vertLimit"] if "vertLimit" in args else 15) < np.rad2deg(angle) < (args["vertLimit"] if "vertLimit" in args else 15):
            (x0, y0) = dist * np.array([np.cos(angle), np.sin(angle)])
            c = -math.sin(angle)
            s = math.cos(angle)

            p1 = (int(x0 - c * 4096), int(y0 - s * 4096))
            p2 = (int(x0 + c * 4096), int(y0 + s * 4096))

            vDisp = cv2.line(vDisp, p1, p2, (0,), thickness=5)

    if verbose:
        print("hough2 start")

    tested_angles = np.linspace(-np.pi / 4, np.pi / 4, 360, endpoint=False)
    h, theta, d = hough_line(vDisp, theta=tested_angles)

    bestAng = 0
    bestDist = 0

    for _, angle, dist in zip(*hough_line_peaks(h, theta, d)):
        if angle < bestAng:
            bestAng = angle
            bestDist = dist

    floorMask = np.ones((disp.shape[0], disp.shape[1]), dtype=np.uint8)

    if verbose:
        print("masking start")

    slope = np.tan(bestAng + np.pi / 2)
    (x0, y0) = bestDist * np.array([np.cos(bestAng), np.sin(bestAng)])

    rows, cols = disp.shape
    for ii in range(rows):
        thresh = ((ii - y0) / slope + x0) + (args["floorThresh"] if "floorThresh" in args else 10)

        for jj in range(cols):
            floorMask[ii, ...] = dispScaled[ii, ...] < thresh
            # if dispScaled[ii, jj] < thresh:
            #     floorMask[ii, jj] = 0

    return (bestAng, bestDist), disp, dispScaled, floorMask
