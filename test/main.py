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




def file_writer():
    global que
    while True:
        if not que.empty():
            que_dict = que.get()
            for key, value in que_dict.items():
                if key == "dis" or key == "slope":
                    np.save(value[0], value[1])
		#elif key == "lidar_data":
		    f = open(value[0],'w',newline='')
		    wr = csv.writer(f)
                    wr.writerows(value[1])
                    f.close()            
		else:
                    cv2.imwrite(value[0], value[1])




def getdegree(angle):
    degree = math.degrees(angle)
    if degree < 0:
        degree = 360 + degree
    degree = int(degree)
    return degree


def record():
    global que, count, zed, runtime, status

    # Mat
    left = sl.Mat()
    right = sl.Mat()
    depth = sl.Mat()
    dis = sl.Mat()

    while True:
        if status:
            if zed.grab(runtime) == sl.ERROR_CODE.SUCCESS:
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
		
                zed.retrieve_image(left, sl.VIEW.LEFT)  # images
                zed.retrieve_image(right, sl.VIEW.RIGHT)
                zed.retrieve_image(depth, sl.VIEW.DEPTH)
                zed.retrieve_measure(dis, sl.MEASURE.DEPTH)  # distance
                path = os.path.join(save_path, frm_path)
                zed.get_sensors_data(sensors_data, sl.TIME_REFERENCE.CURRENT)
                quaternion = sensors_data.get_imu_data().get_pose().get_orientation().get()
		
		
                # Save image
                que.put({'left': [os.path.join(path, f'left_{str(count).zfill(4)}.jpg'), left.get_data()]})
                que.put({'right': [os.path.join(path, f'right_{str(count).zfill(4)}.jpg'), right.get_data()]})
                que.put({'depth': [os.path.join(path, f'depth_{str(count).zfill(4)}.jpg'), depth.get_data()]})
                que.put({'dis': [os.path.join(path, f'distance_{str(count).zfill(4)}.npy'),
                                 DepthNormalizing(distance_undefined(dis.get_data()))]})
                que.put({'slope': [os.path.join(path, f'slope_{str(count).zfill(4)}.npy'), quaternion]})
		que.put({'lidar_data': [os.path.join(path, f'lidar_data_{str(count).zfill(4)}.xyz'), ang]})
                count += 1
        else:
            count = 0


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
                    print("Angle: ", int(degree), " Range: ", str(point.range), " Intensity: ", str(point.intensity)) 
            else:
                print('Failed to get Lidar Data')
	    f = open(value[0],'w',newline='') #원본을 훼손할 위험이 있으니 다른 파일에 저장하는 것을 추천합니다.
	    wr = csv.writer(f)
	    wr.writerows(value[1])
	    f.close()      


if __name__ == "__main__":
    # thread = threading.Thread(target=playLidar, args=())
    # thread.start()
    # thread.join()
    playLidar()
