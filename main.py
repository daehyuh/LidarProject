import os
import ydlidar
import time
import sys
import math
import json
from datetime import datetime
import threading

save_path = 'saved_imgs'

ydlidar.os_init();
ports = ydlidar.lidarPortList();
port = "/dev/ttyS3";
for key, value in ports.items():
    port = value;
laser = ydlidar.CYdLidar();
laser.setlidaropt(ydlidar.LidarPropSerialPort, port);
laser.setlidaropt(ydlidar.LidarPropSerialBaudrate, 230400);
laser.setlidaropt(ydlidar.LidarPropLidarType, ydlidar.TYPE_TRIANGLE);
laser.setlidaropt(ydlidar.LidarPropDeviceType, ydlidar.YDLIDAR_TYPE_SERIAL);
laser.setlidaropt(ydlidar.LidarPropScanFrequency, 12.0);
laser.setlidaropt(ydlidar.LidarPropSampleRate, 5);
laser.setlidaropt(ydlidar.LidarPropSingleChannel, False);
laser.setlidaropt(ydlidar.LidarPropMaxAngle, 180.0);
laser.setlidaropt(ydlidar.LidarPropMinAngle, -180.0);
laser.setlidaropt(ydlidar.LidarPropMaxRange, 16.0);
laser.setlidaropt(ydlidar.LidarPropMinRange, 0.28);
laser.setlidaropt(ydlidar.LidarPropIntenstiy, True);

file_path = "./sample.json"
data = {}
data['data'] = []


def file_make():
    frm_path = datetime.today().strftime('%Y%m%d_%H%M%S%f')
    root_path = os.path.join(save_path, frm_path)
    if not os.path.exists(root_path):
        os.makedirs(root_path)
    data['data'].append({
        "angle": angle,
        "range": range,
        "intensity": intensity
    })
    json_string = json.dumps(json_object)
    print(json_string)


def getdegree(angle):
    degree = math.degrees(angle)
    if degree < 0:
        degree = 360 + degree
    degree = int(degree)
    return degree


def playLidar():
    ret = laser.initialize()
    if ret:
        ret = laser.turnOn()
        scan = ydlidar.LaserScan()
        while ret and ydlidar.os_isOk():
            r = laser.doProcessSimple(scan)
            if r:
                ang = []
                ran = []
                intensity = []
                last_degree = 0
                for point in scan.points:
                    degree = getdegree(point.angle)
                    if degree != last_degree:
                        print(degree, last_degree)
                        ang.append(degree)
                        ran.append(point.range)
                        intensity.append(point.intensity)
                    last_degree = degree
                    print("Angle: ", int(degree), " Range: ", str(point.range), " Intensity: ", str(point.intensity))

            else:
                print('Failed to get Lidar Data')


if __name__ == "__main__":
    # thread = threading.Thread(target=playLidar, args=())
    # thread.start()
    # thread.join()
    playLidar()
