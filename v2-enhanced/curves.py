import math


class curve:
    def renderPoints(self):
        """returns array of points to render"""
        pass

    def getLength(self):
        """returns length of curve"""
        pass

    def getPosDir(self, distance, offset):
        """returns array of positions, and the angle at points defined by offset + n * distance cm"""
        pass


class quadBezier(curve):
    length = None

    def __init__(self, p1: tuple, p2: tuple, p3: tuple, calcSteps=100):
        self.p1 = p1
        self.p2 = p2
        self.p3 = p3
        self.calcSteps = calcSteps

    def updatePoints(self, p1: tuple, p2: tuple, p3: tuple):
        self.p1 = p1
        self.p2 = p2
        self.p3 = p3
        self.length = None

    def renderPoints(self):
        """returns array of points to render"""

        res = []
        self.length = 0.0

        for tt in range(0, self.calcSteps):
            t = tt / self.calcSteps

            x = (1 - t) ** 2 * self.p1[0] + 2 * (1 - t) * t * self.p2[0] + t ** 2 * self.p3[0]
            y = (1 - t) ** 2 * self.p1[1] + 2 * (1 - t) * t * self.p2[1] + t ** 2 * self.p3[1]

            if tt > 0:
                self.length = self.length + math.sqrt((x - res[tt - 1][0]) ** 2 + (y - res[tt - 1][1]) ** 2)

            res.append([x, y])

        return res

    def getLength(self):
        """returns length of curve"""

        if self.length is None:
            tmp = []
            self.length = 0.0

            for tt in range(0, self.calcSteps):
                t = tt / self.calcSteps

                x = (1 - t) ** 2 * self.p1[0] + 2 * (1 - t) * t * self.p2[0] + t ** 2 * self.p3[0]
                y = (1 - t) ** 2 * self.p1[1] + 2 * (1 - t) * t * self.p2[1] + t ** 2 * self.p3[1]

                if tt > 0:
                    self.length = self.length + math.sqrt((x - tmp[0]) ** 2 + (y - tmp[1]) ** 2)

                tmp = [x, y]

        return self.length

    def getPosDir(self, distance, offset):
        """returns array of positions, and the angle at points defined by offset + n * distance cm"""
        tmpLen = self.getLength()
        # print("len " + str(tmpLen))
        curLen = 0.0

        tmp = []

        resPos = []
        resAngle = []

        for tt in range(0, int(math.ceil(tmpLen * 4))):
            t = tt / (tmpLen * 4)

            x = (1 - t) ** 2 * self.p1[0] + 2 * (1 - t) * t * self.p2[0] + t ** 2 * self.p3[0]
            y = (1 - t) ** 2 * self.p1[1] + 2 * (1 - t) * t * self.p2[1] + t ** 2 * self.p3[1]

            toAdd = 0

            if offset == 0 and tt == 0:
                deltaX = 2 * 1 * (self.p2[0] - self.p1[0])  # + 2*tmp[2]*(self.p3[0]-self.p2[0])
                deltaY = 2 * 1 * (self.p2[1] - self.p1[1])  # + 2*tmp[2]*(self.p3[1]-self.p2[1])
                resAngle.append(math.atan2(deltaY, deltaX))

            if tt > 0:
                toAdd = math.sqrt((x - tmp[0]) ** 2 + (y - tmp[1]) ** 2)

                curToDist = min((curLen - offset) % distance, distance - (curLen - offset) % distance)
                nextToDist = min((curLen + toAdd - offset) % distance, distance - (curLen + toAdd - offset) % distance)
                prevToDist = min((curLen - tmp[3] - offset) % distance,
                                 distance - (curLen - tmp[3] - offset) % distance)

                if curToDist < nextToDist and prevToDist > curToDist:
                    resPos.append([tmp[0], tmp[1]])
                    # print("curLen: " + str(curLen) + "     one: " + str(min((curLen-offset)%distance, distance - (curLen-offset)%distance)) + " vs two: " + str(min((curLen+toAdd-offset)%distance, distance - (curLen+toAdd-offset)%distance)))
                    deltaX = 2 * (1 - tmp[2]) * (self.p2[0] - self.p1[0]) + 2 * tmp[2] * (self.p3[0] - self.p2[0])
                    deltaY = 2 * (1 - tmp[2]) * (self.p2[1] - self.p1[1]) + 2 * tmp[2] * (self.p3[1] - self.p2[1])
                    # print(str(deltaX) + ", " + str(deltaY) + " = " + str(math.atan2(deltaY, deltaX)))
                    resAngle.append(math.atan2(deltaY, deltaX))

                curLen += toAdd

            tmp = [x, y, t, toAdd]

        return resPos, resAngle
