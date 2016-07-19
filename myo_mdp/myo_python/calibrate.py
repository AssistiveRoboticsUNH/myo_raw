#!/usr/bin/env python
import time
import rospy
from geometry_msgs.msg import Quaternion, Vector3
from myo_raw.msg import IMUData
import json
import sys
import threading

class Calibrator(object):
    def __init__(self):
        self.time0 = time.time()
        self.imu_quat = []
        self.myo_quat = []
        self.imu_ort = None
        self.myo_ort = None

    def writeIMU(self, msg):
        self.imu_ort = msg

    def writeMyo(self, msg):
        self.myo_ort = msg

    def listener(self):

        rospy.Subscriber('/myo/l/ort', Quaternion, self.writeIMU, queue_size=10)
        rospy.Subscriber('/myo/l/imu', IMUData, self.writeMyo, queue_size=10)
        rospy.spin()

    def record(self):
        count = 0
        while count<50:
            # time1 = time.time()
            # if time1 - self.time0 > 0.5:
            print self.imu_ort
            print self.myo_ort
            if self.imu_ort is not None:
                self.imu_quat.append((self.imu_ort.x,
                                      self.imu_ort.y,
                                      self.imu_ort.z,
                                      self.imu_ort.w))
            if self.myo_ort is not None:
                self.myo_quat.append((self.myo_ort.orientation.x,
                                      self.myo_ort.orientation.y,
                                      self.myo_ort.orientation.z,
                                      self.myo_ort.orientation.w))
            # self.time0 = time1
            count += 1
            print "# data points: ", count
            time.sleep(0.5)

        json.dump(self.imu_quat, open('IMU_sensor.dat', 'w'))
        json.dump(self.myo_quat, open('myo_sensor.dat', 'w'))
        sys.exit(0)

if __name__ == '__main__':

    calibrator = Calibrator()
    rospy.init_node('collect_quat')

    try:
        threading.Thread(target=calibrator.listener).start()
    except:
        print "Could not start thread. ", sys.exc_info()[0]
    calibrator.record()

