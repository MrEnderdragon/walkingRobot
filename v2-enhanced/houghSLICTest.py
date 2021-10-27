import numpy as np
import cv2
from skimage.transform import hough_line, hough_line_peaks
import slic
from skimage.feature import canny
from skimage.draw import line
from skimage import data
import math
import matplotlib.pyplot as plt
from matplotlib import cm

if __name__ == "__main__":

    for i in range(10):
        # Constructing test image
        imgId = "SPLR-" + str(i) + ".png"
        image = cv2.imread("vDisp/v" + imgId, cv2.IMREAD_UNCHANGED)
        dispRaw = cv2.imread("dispImages/depth" + imgId, cv2.IMREAD_UNCHANGED)
        disp = np.copy(dispRaw)
        colour = cv2.imread("RImages/R" + imgId, cv2.IMREAD_COLOR)

        colSmall = cv2.resize(colour, (int(disp.shape[1] / 1), int(disp.shape[0] / 1)), interpolation=cv2.INTER_AREA)

        print("preprocc start")

        pixLab, _, cc = slic.SLIC(colSmall, 300, 30, 5)

        colAvgs = np.zeros(len(cc))
        colAms = np.zeros(len(cc))

        pixLab = cv2.resize(pixLab, (disp.shape[1], disp.shape[0]), interpolation=cv2.INTER_AREA)

        rows, cols = disp.shape
        for row in range(rows):
            for col in range(cols):
                if disp[row, col] != 0:
                    colAvgs[pixLab[row, col]] += disp[row, col]
                    colAms[pixLab[row, col]] += 1

        for ccInd in range(colAvgs.shape[0]):
            colAvgs[ccInd] = 0 if colAms[ccInd] < 100 else int(colAvgs[ccInd] / colAms[ccInd])

        for row in range(rows):
            for col in range(cols):
                if disp[row, col] == 0:
                    disp[row, col] = colAvgs[pixLab[row, col]]

        disp = cv2.convertScaleAbs(disp*10, alpha=(255.0/65535.0))
        dispRaw = cv2.convertScaleAbs(dispRaw*10, alpha=(255.0/65535.0))

        print("preprocc done")

        # Classic straight-line Hough transform
        # Set a precision of 0.5 degree.
        tested_angles = np.linspace(-np.pi / 4, np.pi / 4, 360, endpoint=False)
        h, theta, d = hough_line(image, theta=tested_angles)

        for _, angle, dist in zip(*hough_line_peaks(h, theta, d)):
            if -15 < np.rad2deg(angle) < 15:
                (x0, y0) = dist * np.array([np.cos(angle), np.sin(angle)])
                c = -math.sin(angle)
                s = math.cos(angle)

                p1 = (int(x0 - c * 4096), int(y0 - s * 4096))
                p2 = (int(x0 + c * 4096), int(y0 + s * 4096))

                image = cv2.line(image, p1, p2, (0,), thickness=5)
                bad = True

        tested_angles = np.linspace(-np.pi / 4, np.pi / 4, 360, endpoint=False)
        h, theta, d = hough_line(image, theta=tested_angles)

        # Generating figure 1
        fig, axes = plt.subplots(1, 3, figsize=(15, 6))
        ax = axes.ravel()

        ax[0].imshow(image, cmap=cm.gray)
        ax[0].set_title('Input image')
        ax[0].set_axis_off()

        angle_step = 0.5 * np.diff(theta).mean()
        d_step = 0.5 * np.diff(d).mean()
        bounds = [np.rad2deg(theta[0] - angle_step),
                  np.rad2deg(theta[-1] + angle_step),
                  d[-1] + d_step, d[0] - d_step]
        ax[1].imshow(np.log(1 + h), extent=bounds, cmap=cm.gray, aspect=1 / 1.5)
        ax[1].set_title('Hough transform')
        ax[1].set_xlabel('Angles (degrees)')
        ax[1].set_ylabel('Distance (pixels)')
        ax[1].axis('image')

        ax[2].imshow(image, cmap=cm.gray)
        ax[2].set_ylim((image.shape[0], 0))
        ax[2].set_axis_off()
        ax[2].set_title('Detected lines')

        bestAng = 0
        bestDist = 0

        for _, angle, dist in zip(*hough_line_peaks(h, theta, d)):
            if angle < bestAng:
                bestAng = angle
                bestDist = dist

        (x0, y0) = bestDist * np.array([np.cos(bestAng), np.sin(bestAng)])
        ax[2].axline((x0, y0), slope=np.tan(bestAng + np.pi / 2))

        disp2 = disp.copy()
        disp2 = cv2.applyColorMap(disp2, cv2.COLORMAP_TWILIGHT)

        rows, cols = disp.shape
        for ii in range(rows):
            slope = np.tan(bestAng + np.pi / 2)
            (x0, y0) = bestDist * np.array([np.cos(bestAng), np.sin(bestAng)])
            thresh = (ii - y0) / slope + x0

            for jj in range(cols):
                if disp[ii, jj] < thresh + 10:
                    disp2[ii, jj] = (0, 255, 0)

        mask = disp < 5
        disp2[mask] = 255

        disp = cv2.applyColorMap(disp, cv2.COLORMAP_TWILIGHT)
        dispRaw = cv2.applyColorMap(dispRaw, cv2.COLORMAP_TWILIGHT)

        cv2.imshow('noFloor', disp2)
        cv2.imshow('floor', disp)
        cv2.imshow('raw', dispRaw)
        cv2.imshow('colour', colour)

        plt.tight_layout()
        plt.show()
