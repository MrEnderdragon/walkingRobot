import hough
import matplotlib.pyplot as plt
import mayavi.mlab
import cv2
import numpy as np

focalLen = 19.6 * 10
baseline = 7.5 * 10

maxSize = 5000
step = int(maxSize / 50)

mid = [maxSize / 10, maxSize / 2, 0]

thresh = (30/1000000)*(step**3)

fig = plt.figure(300)
ax = plt.axes()

plt.figure(200)
ax2 = plt.axes(projection='3d')
ax2.set_xlabel('x')
ax2.set_ylabel('y')
ax2.set_zlabel('z')

plt.figure(100)
axobj = plt.axes(projection='3d')
axobj.set_xlabel('x')
axobj.set_ylabel('y')
axobj.set_zlabel('z')


def mapPoints(inX, inY, depth):
    if depth == 0:
        return None

    # hypLen = math.sqrt(inX**2 + inY**2 + focalLen**2)
    conv = depth/focalLen
    return [inX * conv, inY * conv, focalLen * conv]


def fits(minVal, maxVal, arr):
    for ind in arr:
        if ind < minVal or ind >= maxVal:
            # pass
            return False
    return True


def scale(coordsIn, midIn):
    coordsIn[0] *= -1
    return np.add(coordsIn, midIn)


def unscale(coordsIn, midIn):
    coordss = np.subtract(coordsIn, midIn)
    coordss[0] *= -1
    return coordss


def eq(in1, in2):
    for ind in range(len(in1)):
        if in1[ind] != in2[ind]:
            return False
    return True


if __name__ == "__main__":

    for i in range(4, 10):
        # Constructing test image
        imgId = "SPLR-" + str(i) + ".png"
        vDisp = cv2.imread("vDisp/vDisp" + imgId, cv2.IMREAD_UNCHANGED)
        disp = cv2.imread("dispImages/disp" + imgId, cv2.IMREAD_UNCHANGED)
        dep = cv2.imread("depImages/depth" + imgId, cv2.IMREAD_UNCHANGED)
        colour = cv2.imread("RImages/R" + imgId, cv2.IMREAD_COLOR)

        print(scale([0, 0, 10], mid))

        # cv2.imshow("vDisp", vDisp*10)
        # cv2.waitKey()

        _, dispSLIC, dispScaled, floor = hough.hough(disp, vDisp, slicc=False, slicRef=colour, verbose=True, m=100, k=500, scl=1.5, its=5)

        depFloorless = dep * floor

        obs = np.zeros(depFloorless.shape[1], dtype=np.float_)

        shellArr = []

        shellGrid = np.zeros((int(maxSize/step)+1, int(maxSize/step)+1, int(maxSize/step)+1), dtype=np.uint8)
        floorGrid = np.zeros((int(maxSize/step)+1, int(maxSize/step)+1, int(maxSize/step)+1), dtype=np.uint8)
        colorGrid = np.zeros((int(maxSize/step)+1, int(maxSize/step)+1, int(maxSize/step)+1), dtype=np.uint32)

        flatGrid = np.zeros((int(maxSize/step)+1, int(maxSize/step)+1, 3), dtype=np.uint8)

        rows, cols = dep.shape
        for row in range(rows):
            print(row)
            for col in range(cols):
                coords = mapPoints(row - rows / 2, col - cols / 2, dep[row, col])

                if coords is not None:
                    coords = scale(coords, mid)

                    obs[col] = max(obs[col], coords[2])
                    if fits(0, maxSize, coords):
                        shellArr.append((coords[0], coords[1], coords[2]))

                        if floor[row, col]:
                            shellGrid[int(coords[0]/step), int(coords[1]/step), int(coords[2]/step)] += 1
                            colorGrid[int(coords[0] / step), int(coords[1] / step), int(coords[2] / step)] += colour[row][col][0]
                        else:
                            floorGrid[int(coords[0]/step), int(coords[1]/step), int(coords[2]/step)] += 1

        shellArr.append((maxSize, maxSize, 0))
        shellArr.append((maxSize, -0, 0))
        shellArr.append((-0, -0, 0))
        shellArr.append((-0, maxSize, 0))

        kernel_size = 5
        kernel = np.ones(kernel_size) / kernel_size
        obsSmooth = np.convolve(obs, kernel, mode='same')

        # print(obs)
        cv2.imshow("floorless", cv2.applyColorMap(cv2.convertScaleAbs(depFloorless*10, alpha=(255.0/65535.0)), cv2.COLORMAP_JET))
        cv2.imshow("slic", cv2.convertScaleAbs(dispSLIC*10, alpha=(255.0/65535.0)))
        cv2.imshow("scaled", dispScaled)
        cv2.imshow("floor", floor*255)
        cv2.imshow("col", colour)
        cv2.imshow("vDisp", vDisp*10)

        ax.plot(-obsSmooth, "k-", markersize=0.5)
        ax.plot(-obs, "go", markersize=0.5)

        X, Y, Z = zip(*shellArr)

        # ox, oy, oz = np.where(np.logical_and(shellGrid > 5, shellGrid < 10))
        # sx, sy, sz = np.where(np.logical_and(shellGrid > 10, shellGrid < 30))
        bx, by, bz = np.where(shellGrid > thresh)
        colorGrid = np.divide(colorGrid, shellGrid)
        cols = colorGrid[bx, by, bz].astype(np.uint8)
        # print(cols)
        print("cols", len(cols))
        print("bx", len(bx))
        cx, cy, cz = np.where(floorGrid > 0)

        objGrid = np.zeros((int(maxSize/step)+1, int(maxSize/step)+1, int(maxSize/step)+1), dtype=np.uint8)

        for voxInd in range(len(bx)):

            unscCoords = unscale([bx[voxInd] * step, by[voxInd] * step, bz[voxInd] * step], mid)

            x = unscCoords[0]
            y = unscCoords[1]
            z = unscCoords[2]

            if -10 < bx[voxInd] * step < 500:
                flatGrid[by[voxInd], bz[voxInd]] = [0, 0, 255]

            xzRatSmall = float(x-0.5)/z
            xzRatBig = float(x+0.5)/z
            yzRatSmall = float(y-0.5)/z
            yzRatBig = float(y+0.5)/z

            for zz in range(int(z)+1, maxSize, step):
                cont = False
                for xx in range(int(xzRatSmall * zz), int(xzRatBig * zz)+step, step):
                    for yy in range(int(yzRatSmall * zz), int(yzRatBig * zz)+step, step):
                        coords = scale([xx, yy, zz], mid)
                        if fits(0, maxSize, coords):
                            objGrid[int(coords[0]/step), int(coords[1]/step), int(coords[2]/step)] += 1
                            cont = True
                            if -10 < coords[0] < 500 and eq(flatGrid[int(coords[1]/step), int(coords[2]/step)], [0, 0, 0]):
                                flatGrid[int(coords[1]/step), int(coords[2]/step)] = [255, 0, 0]
                if not cont:
                    break

        for voxInd in range(len(cx)):
            unscCoords = unscale([cx[voxInd] * step, cy[voxInd] * step, cz[voxInd] * step], mid)

            x = unscCoords[0]
            y = unscCoords[1]
            z = unscCoords[2]

            if -10 < cx[voxInd] * step < 500 and eq(flatGrid[cy[voxInd], cz[voxInd]], [0, 0, 0]):
                flatGrid[cy[voxInd], cz[voxInd]] = [0, 255, 0]

            xzRatSmall = float(x - 0.5) / z
            xzRatBig = float(x + 0.5) / z
            yzRatSmall = float(y - 0.5) / z
            yzRatBig = float(y + 0.5) / z

            for zz in range(int(z) + 1, maxSize, step):
                cont = False
                for xx in range(int(xzRatSmall * zz), int(xzRatBig * zz)+step, step):
                    for yy in range(int(yzRatSmall * zz), int(yzRatBig * zz)+step, step):
                        coords = scale([xx, yy, zz], mid)
                        if fits(0, maxSize, coords):
                            objGrid[int(coords[0] / step), int(coords[1] / step), int(coords[2] / step)] += 1
                            cont = True
                if not cont:
                    break

        ox, oy, oz = np.where(np.logical_and(floorGrid == 0, np.logical_and(objGrid > 0, shellGrid <= thresh)))

        ax2.plot(xs=X, ys=Y, zs=Z, color='green', marker='o', markersize=0.1, linewidth=0)
        # axobj.plot(xs=objCoords[0], ys=objCoords[1], zs=objCoords[2], color='red', marker='o', markersize=1, linewidth=0)
        # axobj.plot(xs=shellCoords[0], ys=shellCoords[1], zs=shellCoords[2], color='green', marker='o', markersize=1, linewidth=0)

        # axobj.voxels(objGrid, facecolors="red", edgecolors="black")
        # axobj.voxels(shellGrid, facecolors="green", edgecolors="black")

        objects = mayavi.mlab.points3d(-oy, -oz, ox, mode="cube", scale_factor=0.8, color=(0.8, 0.9, 0.8), opacity=0.5)
        # nodes2 = mayavi.mlab.points3d(sx, sy, sz, mode="cube", scale_factor=0.8, color=(0, 1, 0))
        shell = mayavi.mlab.points3d(-by, -bz, bx, mode="cube", scale_factor=0.8, color=(0, 0, 1))
        floor = mayavi.mlab.points3d(-cy, -cz, cx, mode="cube", scale_factor=0.8, color=(1, 0, 0))
        # mayavi.mlab.outline(color=(0, 0, 0))

        shell.glyph.scale_mode = 'scale_by_vector'
        # shell.glyph.color_mode = "color_by_scalar"

        # shell.mlab_source.dataset.point_data.scalars = cols

        cv2.imshow("2d project", flatGrid)

        mayavi.mlab.show()

        # plt.show()

