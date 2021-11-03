import cv2
import numpy as np


def uvDisp(img, max_disp=200):
    height = img.shape[0]
    width = img.shape[1]

    img = cv2.convertScaleAbs(img*10, alpha=(255.0/65535.0))

    mask = np.zeros((height, width, 1), dtype="uint8")+255
    mask[img < 5] = 0

    # V-disparity
    vhist_vis = np.zeros((height, max_disp), float)
    for j in range(height):
        vhist_vis[j, ...] = cv2.calcHist(images=[img[j, ...]], channels=[0], mask=mask[j, ...], histSize=[max_disp],
                                         ranges=[0, max_disp]).flatten() / float(height)

    vhist_vis = np.array(vhist_vis * 255, np.uint8)
    vblack_mask = vhist_vis < 5

    vhist_vis[vblack_mask] = 0

    # U-disparity
    uhist_vis = np.zeros((max_disp, width), float)
    for j in range(width):
        uhist_vis[..., j] = cv2.calcHist(images=[img[..., j]], channels=[0], mask=mask[:, j, :], histSize=[max_disp],
                                         ranges=[0, max_disp]).flatten() / float(width)

    uhist_vis = np.array(uhist_vis * 255, np.uint8)
    ublack_mask = uhist_vis < 5

    uhist_vis[ublack_mask] = 0

    return vhist_vis, uhist_vis
