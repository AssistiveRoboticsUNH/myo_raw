#!/usr/bin/env python

import rospy
from std_msgs.msg import Header
from geometry_msgs.msg import Quaternion, Vector3, Pose, PoseStamped, Point
from myo_raw.msg import Gesture, EMGData, MyoArm, IMUData
from myo.myo import Myo, NNClassifier
from myo.myo_raw import MyoRaw
import tf
import ctypes
import rospkg
import os
from math import pi

# Scale values for unpacking IMU data
# Accelerometer data. In units of g. Range of + -16. Values are multiplied by MYOHW_ACCELEROMETER_SCALE.
# Gyroscope data. In units of deg/s. Range of + -2000. Values are multiplied by MYOHW_GYROSCOPE_SCALE.
MYOHW_ORIENTATION_SCALE = 16384.0 
MYOHW_ACCELEROMETER_SCALE = 2048.0
MYOHW_GYROSCOPE_SCALE = 16.0

class MyoNode(object):
    """A ros wrapper for myo_raw"""
    def __init__(self):
        rospy.init_node('myo_node', anonymous=True)

        traning_data_directory = rospy.get_param('~training_data_dir', None)

        self.pub_imu     = rospy.Publisher('/myo/imu',     IMUData,     queue_size=10)
        self.pub_emg     = rospy.Publisher('/myo/emg',     EMGData,     queue_size=10)
        self.pub_pose    = rospy.Publisher('/myo/gesture', Gesture,     queue_size=10)
        self.pub_myo_arm = rospy.Publisher('/myo/arm',     MyoArm,      queue_size=10, latch=True)
        self.pub_ort     = rospy.Publisher('/myo/ort',     Quaternion,  queue_size=10)

        os.chdir(rospkg.RosPack().get_path('myo_raw')+'/training_data' if traning_data_directory == None else traning_data_directory)
        self.m = Myo(NNClassifier())

    def connect(self):
        self.m.connect()
        self.m.vibrate(2)
        self.m.add_emg_handler(self.__on_emg)
        self.m.add_imu_handler(self.__on_imu)
        # self.m.add_pose_handler(self.__on_pose)
        self.m.add_raw_pose_handler(self.__on_raw_pose)
        self.m.add_arm_handler(self.__on_arm)

        self.baseRot = None 
    
    def disconnect(self):
        self.m.disconnect()


    
    # builtin pose subscriber
    # def __on_pose(self, p):
    #     self.pub_pose.publish(is_builtin=True, pose_number=p, confidence=0.0)

    def __on_arm(self, arm, xdir):
        self.pub_myo_arm.publish(arm=arm, xdir=xdir)
    
    def __on_emg(self, emg, moving):
        self.pub_emg.publish(emg_data=emg, moving=moving)

    def __on_raw_pose(self, p):
        self.pub_pose.publish(is_builtin=False, pose_number=p, confidence=0.0)

    def __on_imu(self, quat, acc, gyro):
        # need to switch the yaw and the roll for some reason
        euler = tf.transformations.euler_from_quaternion((quat[0], quat[1], quat[2], quat[3])) # roll, pitch, yaw
        
        self.pub_imu.publish(header=Header(frame_id=rospy.get_param('frame_id', 'map'), stamp=rospy.get_param('stamp', None)),
                               orientation=Vector3(x=euler[0]*180/pi, y=euler[1]*180/pi, z=euler[2]*180/pi),
                               angular_velocity=Vector3(x=gyro[0], y=gyro[1], z=gyro[2]),
                               linear_acceleration=Vector3(x=acc[0]/MYOHW_ACCELEROMETER_SCALE, y=acc[1]/MYOHW_ACCELEROMETER_SCALE, z=acc[2]/MYOHW_ACCELEROMETER_SCALE)
                                                      )

        if self.baseRot == None:
            self.baseRot = euler[0]
                

        # need to switch the yaw and the roll for some reason   2    1     0
        rotated_quat = tf.transformations.quaternion_from_euler(-euler[2], euler[1], -euler[0] + self.baseRot)
        self.pub_ort.publish(Quaternion(x=rotated_quat[0], 
                                                    y=rotated_quat[1], 
                                                    z=rotated_quat[2], 
                                                    w=rotated_quat[3]))

    def run(self): # note this function is EXTREAMELY time sensitive... delay will cause a disconnect of the myo
        try:    
            while not rospy.is_shutdown():
                self.m.run()
        except rospy.ROSInterruptException:
            self.m.disconnect()

        print "myo node shuttting down"
        self.m.disconnect()

if __name__ == '__main__':
    m = MyoNode()
    m.connect()
    m.run()
