import os
import ydlidar
import time
import sys
import math
import json
from datetime import datetime
import threading
from queue import Queue
import csv

que = Queue()

save_path = 'saved_imgs'

ydlidar.os_init();
ports = ydlidar.lidarPortList();
port = "/dev/ttyUSB4";
for key, value in ports.items():
    port = value;
laser = ydlidar.CYdLidar();
laser.setlidaropt(ydlidar.LidarPropSerialPort, "/dev/ttyUSB0");
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
                last_degree = 0
                for point in scan.points:
                    degree = getdegree(point.angle)
                    if degree != last_degree:
                        print(degree, last_degree)
                        ang.append([degree, point.range, point.intensity])
                        last_degree = degree
                    else:
                        print('Failed to get Lidar Data')
                frm_path = datetime.today().strftime('%Y%m%d_%H%M%S%f')
                root_path = os.path.join(save_path, frm_path)
                os.makedirs(root_path, exist_ok=True)  # image dir
                print(root_path)
                
                ang.sort(key=lambda x:x[0])
                # print(ang)
                saved_imgs/20220603_101837604823 + /data.csv
                with open(root_path+'/data.csv','w', newline='') as f:
                    wr = csv.writer(f)
                    wr.writerows(ang)
                    f.close()


if __name__ == "__main__":
    # thread = threading.Thread(target=playLidar, args=())
    # thread.start()
    # thread.join()
    playLidar()
