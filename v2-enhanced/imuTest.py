import math
import time
import numpy as np
from scipy.spatial.transform import Rotation as R


def euler_from_quaternion(i, j, k, real):
    """
    Convert a quaternion into euler angles (roll, pitch, yaw)
    roll is rotation around x in radians (counterclockwise)
    pitch is rotation around y in radians (counterclockwise)
    yaw is rotation around z in radians (counterclockwise)
    """

    # roll = np.arctan2(j*k + real*i, 1/2 - (i*i+j*j))
    # pitch = np.arcsin(-2*(i*k - real*j))
    # heading = np.arctan2(i*j + real*k, 1/2 - (j*j+k*k))
    #
    # return np.rad2deg(roll), np.rad2deg(pitch), np.rad2deg(heading)

    return R.from_quat([i, j, k, real]).as_euler('yxz', degrees=True)


def readGyros(q):

    imuData = q.get()
    imuPackets = imuData.packets

    rVvalues = imuPackets[len(imuPackets)-1].rotationVector

    res = euler_from_quaternion(rVvalues.i, rVvalues.j, rVvalues.k, rVvalues.real)

    return res


if __name__ == "__main__":
    import depthai as dai

    pipeline = dai.Pipeline()
    imu = pipeline.create(dai.node.IMU)
    xlinkOut = pipeline.create(dai.node.XLinkOut)

    xlinkOut.setStreamName("imu")

    # enable RAW_ACCELEROMETER and RAW_GYROSCOPE at 100 hz rate
    imu.enableIMUSensor([dai.IMUSensor.ROTATION_VECTOR], 100)
    # above this threshold packets will be sent in batch of X, if the host is not blocked and USB bandwidth is available
    imu.setBatchReportThreshold(1)
    # maximum number of IMU packets in a batch, if it's reached device will block sending until host can receive it
    # if lower or equal to batchReportThreshold then the sending is always blocking on device
    # useful to reduce device's CPU load  and number of lost packets, if CPU load is high on device side due to multiple nodes
    imu.setMaxBatchReports(10)

    imu.out.link(xlinkOut.input)

    print("init finish")

    with dai.Device(pipeline) as device:

        imuQueue = device.getOutputQueue(name="imu", maxSize=50, blocking=False)
        print("q got")

        form = "{price:.2f}"

        while True:
            got = readGyros(imuQueue)
            x = form.format(price=got[0])
            y = form.format(price=got[1])
            z = form.format(price=got[2])

            print(x + (" " * (10-len(x))) + y + (" " * (10-len(y))) + z)

            time.sleep(0.1)
