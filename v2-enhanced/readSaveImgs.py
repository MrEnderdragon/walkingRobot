import numpy as np
import cv2

if __name__ == "__main__":

    i = 1

    imgId = "CAL-SPLR-" + str(i) + ".png"
    disp = cv2.imread("dispImages/disp" + imgId, cv2.IMREAD_UNCHANGED)
    depth = cv2.imread("depImages/depth" + imgId, cv2.IMREAD_UNCHANGED)

    dd = cv2.applyColorMap(cv2.convertScaleAbs(depth * 10, alpha=(255.0 / 65535.0)), cv2.COLORMAP_JET)

    cv2.imwrite('./displayImgs/depth' + str(imgId), dd)