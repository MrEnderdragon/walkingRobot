import cv2
import numpy as np
from numpy.random import *


def draw_object(img, x, y, w=50, h=100):
    color = img[y, x]
    img[y - h:y, x - w // 2:x + w // 2] = color


if __name__ == '__main__':
    height = 400
    width = 640
    max_disp = 200
    for i in range(10):
        img = cv2.imread("dispImages/depthSPLR-" + str(i) + ".png", cv2.IMREAD_UNCHANGED)

        img = cv2.convertScaleAbs(img*10, alpha=(255.0/65535.0))

        # print(img.dtype)

        mask = np.zeros((height, width, 1), dtype="uint8")+255
        mask[img < 5] = 0

        # V-disparity
        vhist_vis = np.zeros((height, max_disp), float)
        for j in range(height):
            vhist_vis[j, ...] = cv2.calcHist(images=[img[j, ...]], channels=[0], mask=mask[j, ...], histSize=[max_disp],
                                             ranges=[0, max_disp]).flatten() / float(height)

        vhist_vis = np.array(vhist_vis * 255, np.uint8)
        vblack_mask = vhist_vis < 5
        # vhist_vis = cv2.applyColorMap(vhist_vis, cv2.COLORMAP_JET)
        vhist_vis[vblack_mask] = 0

        # mask = np.zeros((width, height, 1), dtype="uint8") * 255
        # mask[img < 5] = 0

        # U-disparity
        uhist_vis = np.zeros((max_disp, width), float)
        for j in range(width):
            uhist_vis[..., j] = cv2.calcHist(images=[img[..., j]], channels=[0], mask=mask[:, j, :], histSize=[max_disp],
                                             ranges=[0, max_disp]).flatten() / float(width)

        uhist_vis = np.array(uhist_vis * 255, np.uint8)
        ublack_mask = uhist_vis < 5
        # uhist_vis = cv2.applyColorMap(uhist_vis, cv2.COLORMAP_JET)
        uhist_vis[ublack_mask] = 0

        # save result
        img = cv2.applyColorMap(img, cv2.COLORMAP_JET)
        cv2.imshow('disparity.png', img)
        cv2.imshow('v_disparity.png', vhist_vis)
        cv2.imshow('u_disparity.png', uhist_vis)
        cv2.imshow('mask.png', mask)

        cv2.imwrite('./vDisp/vSPLR-' + str(i) + '.png', vhist_vis)
        cv2.imwrite('./uDisp/uSPLR-' + str(i) + '.png', uhist_vis)

        cv2.waitKey()