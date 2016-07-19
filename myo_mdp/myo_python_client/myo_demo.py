'''
Created on Jan 20, 2016

@author: ymeng
'''
#!/usr/bin/env python
import rospy
from std_msgs.msg import Empty
from numpy import genfromtxt
from geometry_msgs.msg import Quaternion
import tf


class MyoDemo(object):
    def __init__(self, use_euler=False):

        self.pub = rospy.Publisher('/exercise/playback', Quaternion, queue_size=20)
    
    
        data = genfromtxt('../data/imu_sample.dat', delimiter=',')
        self.quat = []
        if use_euler:
            orientation = data[:, 6:9]
            #print orientation.shape
            for euler in orientation:
                rotated_quat = tf.transformations.quaternion_from_euler(euler[2], euler[1], euler[0])
                self.quat.append(Quaternion(x=rotated_quat[0], 
                                  y=rotated_quat[1], 
                                  z=rotated_quat[2], 
                                  w=rotated_quat[3]))
            
        else:
            orientation = data[:, -4:]
            for q in orientation:
                self.quat.append(Quaternion(x=q[0], y=q[1], z=q[2], w=q[3]))
        
        self.quat.append(Quaternion(x=-1337, y=-1337, z=-1337, w=-1337))
        
        

    def callback(self, message):
        counter = 0
        rate = rospy.Rate(50) # 50hz
        for q in self.quat:
            self.pub.publish(q)
            counter += 1
            rate.sleep()
        print counter, "coordinates pulished."
        
    def subscriber(self):
        rospy.init_node('listener', anonymous=True)
        rospy.Subscriber('/exercise/playback_trigger', Empty, self.callback)
        rospy.spin()
    
if __name__ == '__main__':
    demo = MyoDemo()
    demo.subscriber()