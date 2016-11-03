#!/usr/bin/env python

import rospy
import tf
import ctypes
import rospkg
import os
import sys
from geometry_msgs.msg import Quaternion, Vector3, Pose, PoseStamped, Point
from myo_raw.msg import Gesture, EMGData, MyoArm, IMUData, EMGIMU
from std_msgs.msg import Header, Int32, String
# from myo_raw_lib import identifiers as id
import identifiers as id
from myo.myo import Myo, SimpleMyo, NNClassifier
from myo.myo_raw import MyoRaw
from threading import Thread
from time import sleep
from math import pi


class SimpleMyoNode(object):
    def __init__(self):
        self.m = SimpleMyo()
        self.connect()

        self.identifier = id.id.get(self.m.device_name)
        self.pub_imu = rospy.Publisher('/myo/' + self.identifier + '/imu', IMUData, queue_size=10)
        self.pub_emg = rospy.Publisher('/myo/' + self.identifier + '/emg', EMGData, queue_size=10)
        self.pub_pose = rospy.Publisher('/myo/' + self.identifier + '/gesture', Gesture, queue_size=10)
        self.pub_myo_arm = rospy.Publisher('/myo/' + self.identifier + '/arm', MyoArm, queue_size=10, latch=True)
        self.pub_ort = rospy.Publisher('/myo/' + self.identifier + '/ort', Quaternion, queue_size=10)
        self.pub_emgimu = rospy.Publisher('/myo/' + self.identifier + '/emgimu', EMGIMU, queue_size=10)
        self.downsampler = 0
        self.baseRot = None
        self.count = 0
        self.keepScanning = True

    def connect(self):
        self.m.connect()
        self.m.vibrate(2)
        self.m.add_emg_handler(self.__on_emg)
        self.m.add_imu_handler(self.__on_imu)
        # m.add_pose_handler(self.__on_pose)
        self.m.add_raw_pose_handler(self.__on_raw_pose)
        self.m.add_arm_handler(self.__on_arm)

    def disconnect(self):
        self.m.disconnect

    def __on_pose(self, p):
        self.pub_pose.publish(is_builtin=True, pose_number=p, confidence=0.0)

    def __on_arm(self, arm, xdir):
        self.pub_myo_arm.publish(arm=arm, xdir=xdir)

    def __on_emg(self, emg, moving):
        self.pub_emg.publish(emg_data=emg, moving=moving)
        self.emg = emg

    def __on_raw_pose(self, p):
        self.pub_pose.publish(is_builtin=False, pose_number=p, confidence=0.0)

    def __on_imu(self, quat, acc, gyro):
        euler = tf.transformations.euler_from_quaternion((quat[0], quat[1], quat[2], quat[3]))

        if self.baseRot is None:
            self.count += 1
            if self.count > 360:
                self.baseRot = euler
            return

        calibrated = -euler[0] + self.baseRot[0]
        if calibrated > pi:
            calibrated -= 2*pi
        if calibrated < -pi:
            calibrated += 2*pi

        exg = -2*(euler[2] - self.baseRot[2]) - self.baseRot[2]
        rotated_quat = tf.transformations.quaternion_from_euler(exg, euler[1], calibrated)

        # if 'myo-only' in sys.argv:
        self.pub_ort.publish(Quaternion(x=rotated_quat[0],
                                        y=rotated_quat[1],
                                        z=rotated_quat[2],
                                        w=rotated_quat[3]))

        self.pub_imu.publish(
            header=Header(frame_id=rospy.get_param('frame_id', 'map'), stamp=rospy.get_param('stamp', None)),
            angular_velocity=Vector3(x=gyro[0], y=gyro[1], z=gyro[2]),
            linear_acceleration=Vector3(x=acc[0] / id.MYOHW_ACCELEROMETER_SCALE, y=acc[1] / id.MYOHW_ACCELEROMETER_SCALE,
                                        z=acc[2] / id.MYOHW_ACCELEROMETER_SCALE),
            orientation=Quaternion(x=rotated_quat[0], y=rotated_quat[1], z=rotated_quat[2], w=rotated_quat[3])
            )

        if self.emg:
            self.pub_emgimu.publish(
                header=Header(frame_id=rospy.get_param('frame_id', 'map'), stamp=rospy.get_param('stamp', None)),
                emg=self.emg,
                angular_velocity=gyro,
                linear_acceleration=[acc[0] / id.MYOHW_ACCELEROMETER_SCALE, acc[1] / id.MYOHW_ACCELEROMETER_SCALE,
                                     acc[2] / id.MYOHW_ACCELEROMETER_SCALE],
                orientation=[rotated_quat[0], rotated_quat[1], rotated_quat[2], rotated_quat[3]]
                )

    def run(self):
        try:
            while not rospy.is_shutdown() and self.keepScanning and not self.m.bt.disconnected:
                self.m.run()
        except rospy.ROSInterruptException or KeyboardInterrupt:
            self.m.disconnect()

        rospy.logwarn("Exiting Because: " + " " + str(not rospy.is_shutdown()) + " " + str(self.keepScanning) + " " + str(not self.m.bt.disconnected))

        self.m.initialized = False

        rospy.logwarn("Simple Myo Node for "+self.m.device_name+" shutting down")



class MyoThread(Thread):

    # list containing one thread for each myo declared in
    # the identifiers list
    t = [None for i in range(0, len(id.id))]

    def __init__(self, myo=None):
        Thread.__init__(self)
        self.m = myo

        if self.m is None:
            self.m = SimpleMyoNode()

    def run(self):
        self.m.run()

    @staticmethod
    def isInitialized(thread):
        return not (thread is None or thread.m is None or not thread.m.m.initialized)

    @staticmethod
    def calibrateMyo(msg):
        if msg.data not in (1, 2,'calibration'):
            return
        for i in range(0, len(MyoThread.t)):
            if MyoThread.isInitialized(MyoThread.t[i]):
                MyoThread.t[i].m.baseRot = None


    @staticmethod
    def allInitialized(count=len(id.id)):
        for i in range(0, count):
            if not (MyoThread.isInitialized(MyoThread.t[i])):
                return False
        return True

    @staticmethod
    def startMyo(myocount=len(id.id)):
        if myocount > len(id.id):
            myocount = len(id.id)
        for i in range(0, myocount):
            if MyoThread.t[i] is None:
                MyoThread.t[i] = MyoThread()
                MyoThread.t[i].daemon = True
                MyoThread.t[i].start();
                sleep(1)
            elif not (MyoThread.t[1].m is None):
                MyoThread.t[i].m.m.initialized = False

        while not MyoThread.allInitialized(myocount):
            for i in range(0, myocount):
                if not MyoThread.isInitialized(MyoThread.t[i]):
                    print("Attempting to initialize Myo "+str(i+1))
                    MyoThread.t[i].m.keepScanning = False
                    sleep(1)
                    MyoThread.t[i].m.disconnect()
                    sleep(1)
                    MyoThread.t[i].join()
                    MyoThread.t[i].m.m.bt.disconnected = False
                    MyoThread.t[i].m.keepScanning = True
                    MyoThread.t[i] = MyoThread(MyoThread.t[i].m)
                    MyoThread.t[i].m.m.connect()
                    MyoThread.t[i].start
                    sleep(1)


def main(msg):
    start_myo_sub = rospy.Subscriber("/myo/launch/", Int32, MyoThread.startMyo)
    calibrate_myo_sub = rospy.Subscriber("/myo/calibrate", Int32, MyoThread.calibrateMyo)
    calibrate_myo_sub_speech = rospy.Subscriber("/exercise/detected_state", String, MyoThread.calibrateMyo)
    rospy.spin()
    for i in range(0, len(MyoThread.t)):
        if not (MyoThread.t[i] is None):
            MyoThread.t[i].join()


if __name__ == '__main__':
    rospy.init_node('simple_myo_node', anonymous=True)
    main(None)
