import cv2
import numpy as np
import obstacleDetect
import aStar
# import time
# import curves
import genPath
import time
import matplotlib.pyplot as plt


robotWidth = 112.6/2+120


def renderImgCoord(inGrid):
    return np.flip(inGrid, axis=0)


def mapVal(inSt, inEn, outSt, outEn, val):
    return (val-inSt)/(inEn-inSt) * (outEn-outSt) + outSt


def processImages(start, end, verbose=False):
    vDisps = []
    disps = []
    deps = []
    # rots = [0, np.deg2rad(90)]
    rots = []

    st = start
    en = end
    
    if verbose:
        fig = plt.figure('image', figsize=(10, 6))

    for i in range(st, en+1):
        # Constructing test image
        # imgId = "ROT-SPLR-" + str(i) + ".png"
        imgId = "16-" + str(i) + ".png"
        vDisp = cv2.imread("vDisp/vDisp" + imgId, cv2.IMREAD_UNCHANGED)
        disp = cv2.imread("dispImages/disp" + imgId, cv2.IMREAD_UNCHANGED)
        dep = cv2.imread("depImages/depth" + imgId, cv2.IMREAD_UNCHANGED)
        # colour = cv2.imread("RImages/R" + imgId, cv2.IMREAD_COLOR)
        
        if verbose:
            colour = cv2.imread("colImages/col-" + str(i) + ".png", cv2.IMREAD_GRAYSCALE)
            depthColour = cv2.imread("depthColor/depth-" + str(i) + ".png", cv2.IMREAD_UNCHANGED)
            fig.add_subplot(3, 3, 3 - (i - st))
            plt.imshow(colour, cmap='gray')
            plt.axis('off')
            plt.title("image " + str(i))        
            fig.add_subplot(2, 3, 6 - (i - st))
            plt.imshow(depthColour)
            plt.axis('off')
            plt.title("depth " + str(i))        

        vDisps.append(vDisp)
        disps.append(disp)
        deps.append(dep)
        rots.append(np.deg2rad(mapVal(st, en, -45, 45, i)))

    if verbose:
        plt.tight_layout()
        plt.show(block=False)

    start = time.time()
    shellFlat, obsFlat, walkFlat, _ = obstacleDetect.detectMult(vDisps, disps, deps, rots, True, False)
    # onPath, path, closestNode, voro, walkmap = \
    #     aStar.aStar(shellFlat, obsFlat, walkFlat, unwalkCoords, (shellFlat.shape[0]/2, shellFlat.shape[1] - 1), verbose=True,
    #                 distFunc=aStar.euclid, goalFunc=aStar.euclid, voroFunc=aStar.euclid, robotWidth=robotWidth,
    #                 ignoreDia=False, start=(int(shellFlat.shape[0] / 2), int(shellFlat.shape[1] / 2)))

    onPath, path, closestNode, voro, walkmap = \
        aStar.aStar(shellFlat, obsFlat, walkFlat, None,
                    verbose=True,
                    distFunc=aStar.euclid, goalFunc=aStar.euclid, voroFunc=aStar.euclid, robotWidth=(robotWidth + 50), voroMax=600, 
                    ignoreDia=False, start=(int(shellFlat.shape[0] / 2), int(shellFlat.shape[1] / 2)))

    # newCurves, curvedpath = genPath.gen_path((onPath * 255).astype(np.uint8))
    newCurves, curvedpath, _ = genPath.gen_path(path)
    
    end = time.time()
    print("overall")
    print(end-start)

    tmp = ""
    
    for i in newCurves:
        for p in i.renderPoints():
            tmp += "(" + str(p[0]) + "," + str(p[1]) + "),"
    
    print(tmp)
    
    if verbose:
        
        fig = plt.figure(333)
        fig.add_subplot(2, 3, 1)
        plt.imshow((shellFlat*255).astype(np.uint8))
        plt.axis('off')
        plt.title("shell")        

        fig.add_subplot(2, 3, 2)
        obsDisp = (obsFlat*255).astype(np.uint8)
        obsDisp[np.where(curvedpath > 0)] = 100
        plt.imshow(obsDisp)
        plt.axis('off')
        plt.title("obs")        

        fig.add_subplot(2, 3, 3)
        plt.imshow((walkFlat*255).astype(np.uint8))
        plt.axis('off')
        plt.title("walk")        

        fig.add_subplot(2, 3, 4)
        plt.imshow((voro * 255/(600/50)).astype(np.uint8))
        plt.axis('off')
        plt.title("voro")        

        fig.add_subplot(2, 3, 5)
        plt.imshow((onPath * 255).astype(np.uint8))
        plt.axis('off')
        plt.title("path")        

        fig.add_subplot(2, 3, 6)
        plt.imshow((curvedpath * 255).astype(np.uint8))
        plt.axis('off')
        plt.title("curvespath")
        plt.show()
        # plt.waitforbuttonpress(0)
        # plt.close('all')
        # cv2.imshow("voro", (voro * 255/(400/50)).astype(np.uint8))
        # cv2.imshow("path", (onPath * 255).astype(np.uint8))
        # cv2.imshow("shell", (shellFlat*255).astype(np.uint8))
        # cv2.imshow("obs", (obsFlat*255).astype(np.uint8))
        # cv2.imshow("walk", (walkFlat*255).astype(np.uint8))
        # cv2.imshow("curvedpath", (curvedpath * 255).astype(np.uint8))
        # cv2.waitKey()

    return newCurves


def takeImage(q, _, camLock, __, camSleepTime, **___):
    for ind in range(59, 62):
        camLock.acquire()
        newCurves = processImages(ind*3, ind*3+2, False)
        q.put(newCurves)
        camLock.release()
        time.sleep(5)
        

if __name__ == "__main__":
    for ind in range(52, 60):
        newCurvess = processImages(ind*3, ind*3+2, True)
