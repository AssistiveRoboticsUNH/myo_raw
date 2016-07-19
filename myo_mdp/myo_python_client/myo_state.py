'''
Created on Jan 14, 2016

@author: ymeng
'''

import cPickle as pickle
import rospy
from std_msgs.msg import String, Float64, Empty
from geometry_msgs.msg import Quaternion
from myo_raw.msg import IMUData, EMGIMU
import classifier
import numpy as np
import sys
import time
import tf
from numpy import pi
from myo_demo import MyoDemo
import time
import math
import preprocess
from scipy.signal import savgol_filter # need scipy version 0.16
from prompt import EMG_WEIGHT, evaluate

state_map = {'s1':'twisting clockwise', 's4':'reaching limit counter-clockwise', 's2':'starting/ending position', 's3':'twisting counter-clockwise'}

class MyoPrompt(MyoDemo):
    def __init__(self):
        super(MyoPrompt, self).__init__()
        
    def callback(self, percentage):
        counter = 0
        starting = int(percentage*len(self.quat))
        rate = rospy.Rate(10) # 50hz
        for q in self.quat[starting:]:
            self.pub.publish(q)
            counter += 1
            rate.sleep()
        print counter, "coordinates pulished."

    def subscriber(self):
        """Do not use subscriber here. 
           The callback function will be called directly without ros message
        """
        pass    

class Progress(object):
    
    def __init__(self, classifier=None, classifier_pkl='../data/state_classifier.pkl', give_prompt=True, **kwargs):
        if classifier:
            self.classifier = classifier
            self.EMG_MAX = kwargs.get('EMG_MAX', 585)
            self.EMG_MIN = kwargs.get('EMG_MIN', 0)
            self.GYRO_MAX = kwargs.get('GYRO_MAX', 500)
            self.baseline = kwargs.get('baseline', 0)
        elif classifier_pkl:
            # self.classifier = classifier.SignalClassifier()
            self.classifier, self.EMG_MAX, self.EMG_MIN, self.GYRO_MAX, self.baseline = pickle.load(open(classifier_pkl))
        
        self.give_prompt = give_prompt
        self.baseRot = None
        self.start = False
        self.history = []
        self.user_id = kwargs.get('id', 'new_user')
        self.mdp = kwargs.get('mdp', None)
        
        self.pub = rospy.Publisher('/exercise/progress', Float64, queue_size=10)
        self.pub1 = rospy.Publisher('/exercise/state', String, queue_size=10)
        self.progress = 0.0
        self.count_down = 10 # repeat the state count_down times to be considered entering the state
        self.delay = 0 # time to give prompt if no progress
        self.prompt_now = False
        self.previous = []
        self.getTask()
        rospy.init_node('prompter')
        if give_prompt:
            self.activatePrompt()
#            self.subscribeTrigger()
            self.subscribeIMU()
        else:
            self.start = True
            self.subscribeIMU()
        rospy.spin()
    
    def activatePrompt(self):
        self.prompt = MyoPrompt()
        rospy.Subscriber('/exercise/playback_trigger', Empty, self.starter)
        print "listening to trigger"
        self.prompt.callback(0)
        self.start = True
            
    def getTask(self):
        with open('../data/state_sequence.dat') as f:
            self.full_task = [x.strip() for x in f]
            self.n_states = len(self.full_task)
            self.task = self.full_task[:] # copy the list
        print self.task
        time.sleep(1)
    
    def callback(self, imu):
        if self.start and not self.prompt_now:
            self.getProgress(imu)
    
    def subscribeIMU(self):
        #rospy.init_node('state_receiver')
        rospy.Subscriber('/myo/emgimu', EMGIMU, self.callback, queue_size=2)
        #rospy.spin()
    
    def starter(self, msg):
        self.prompt.callback(0)
        self.start = True
    
    def subscribeTrigger(self):
        rospy.Subscriber('/exercise/playback_trigger', Empty, self.starter)
        print "listening to trigger"
    
    def getProgress(self, emgimu):
        #=======================================================================
        # if self.baseRot is None:
        #     self.baseRot = imu.orientation.x * pi/180 # used for calibration
        #=======================================================================
        #quat = tf.transformations.quaternion_from_euler(-imu.orientation.z, imu.orientation.y, -imu.orientation.x + self.baseRot)
#===============================================================================
#         imu_array = np.array([imu.linear_acceleration.x, imu.linear_acceleration.y, imu.linear_acceleration.z,
#                       imu.angular_velocity.x, imu.angular_velocity.y, imu.angular_velocity.z,
#                       imu.orientation.x, imu.orientation.y,imu.orientation.z, imu.orientation.w])
# #        print imu_array
#         #imu_array[3:6] = imu_array[3:6]/500
#         imu_array[3:6] = process_gyro(imu_array[3:6])
#===============================================================================
        
        emg = preprocess.process_emg(np.array(emgimu.emg), self.EMG_MAX, self.EMG_MIN)
        acc = np.array(emgimu.linear_acceleration)
        gyro = preprocess.process_gyro(np.array(emgimu.angular_velocity), self.GYRO_MAX, discrete=False)
        orie = np.array(emgimu.orientation)
        
        signal_array = np.hstack((EMG_WEIGHT*emg, acc, gyro, orie))
        self.history.append(signal_array)
        if len(self.previous)>10: # 0.2 seconds
            self.previous.pop(0)
        self.previous.append(signal_array)
        current_signal = np.mean(self.previous, axis=0)
            

        #imu_array = np.array([imu.x, imu.y, imu.z, imu.w])
        #print current_signal
        #state = 's'+str(int(self.classifier.predict(imu_array)[0]))
        state = 's'+str(int(self.classifier.predict(current_signal)[0]))
        self.pub1.publish(state)
        self.pub.publish(self.progress)
        if self.task == []:
            print "Task completed!"
            self.pub.publish(1.0)
            self.task = self.full_task[:]
            self.start = False
            self.progress = 0
            self.evaluate_pfmce()
            
            #sys.exit()
        if state == self.task[0]:
            self.count_down -= 1
            if self.count_down == 0:
                # print "current state", state, state_map[state]
                self.task.pop(0)
                self.progress = 1 - 1.0*len(self.task)/(self.n_states-1)
                self.pub.publish(self.progress)
                self.count_down = 10
                self.delay = 0
        elif self.give_prompt:
            # no transition
            self.delay += 1
            if len(self.task)>0 and self.delay>500:
                #self.start = False
                self.prompt_now = True
                print "prompt started"
                print self.prompt_now
                self.prompt.callback(self.progress)
                print "prompt ended"
                self.delay = 0
                self.prompt_now = False
                
    def evaluate_pfmce(self):
        print "Evaluating performance...."
        if not self.mdp:
            print "No MDP found"
            self.history = []
            return
        action_classifier = pickle.load(open('../data/action_classifier.pkl'))
        history = np.array(self.history)
        history = history[25:-6:5, :] # downsampling, cut the first half second
        history = savgol_filter(history, 31, 3, axis=0) # smoothing
        actions = []
        states = []
        for signal in history:
            emg = signal[0:8]/EMG_WEIGHT
            actions.append(int(action_classifier.predict(emg)[0]))
            states.append(int(self.classifier.predict(signal)[0]))
        print actions
        print states
        result = evaluate(actions, states, self.mdp)
        print result
        score = 100*math.exp((result-self.baseline)/200)
        print "Performance score =", int(score)
        np.savetxt('user_data/'+self.user_id, history, delimiter=',')
        self.history = []     
                
                
            
            

if __name__ == '__main__':
    progress = Progress(give_prompt=True)

    