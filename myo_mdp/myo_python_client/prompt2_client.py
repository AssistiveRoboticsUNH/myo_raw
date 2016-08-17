#!/usr/bin/env python

'''
Created on Mar 24, 2016

@author: ymeng
'''
import myo_state2
from myo_demo2 import MyoDemo2
import cPickle as pickle
import time
from geometry_msgs.msg import Quaternion
from std_msgs.msg import Header, Int32, String
import rospy
    
progress = None
task = None
demo = None
n_repeats = 0
nrepeats_pub = rospy.Publisher('/exercise/n_repeats', Int32, queue_size=1)

def signal_handler(msg):
    """ Speech and button controls share the same handler here"""
    
    print "signal received", msg

    global progress, task, demo, n_repeats

    if msg.data in (10, 20, 30):
        task_type = msg.data/10
        if progress and not progress.finished:
            progress.end_game()
 
        if task != msg.data:
            n_repeats = 0
            nrepeats_pub.publish(n_repeats)
            demo = myo_state2.MyoPrompt2(task_type=task_type)
            time.sleep(.5)
            demo.callback(0, 1)
            
            mdp = pickle.load(open('../data'+str(task_type)+'/mdp.pkl'))
            args = {"give_prompt": True,
                    "mdp": mdp,
                    "id": "new patient",
                    "task_type": task_type
                    }
            progress = myo_state2.Progress(classifier_pkl='../data'+str(task_type)+'/state_classifier.pkl', **args)
        else:
            n_repeats += 1
            nrepeats_pub.publish(n_repeats)
            #time.sleep(0.5)
            progress.reset()
            progress.finished = False # start tracking proress
        task = msg.data

    elif msg.data == -1:
        progress.pub.publish(1.0)
        progress.end_game()

    elif msg.data == 100:
        # "Home" command
        if demo is not None: 
            demo.skip = True
        if progress is not None:
            progress.end_game()
        n_repeats = 0
        progress = None
        task = None
        demo = None
        
    elif msg.data == -2:
        # skip demonstration
        if demo is not None:
            print "skipping demo..."
            demo.skip = True
        
def speech_handler(msg):
    print "Message received: ", msg.data
    
    # using 10s because single digits are used by other messages in the trianing interface.
    if msg.data == 'task one':
        signal_handler(Int32(10))
    if msg.data == 'task two':
        signal_handler(Int32(20))
    if msg.data == 'task three':
        signal_handler(Int32(30))



if __name__ == '__main__':
    
    print "Running client mode..."
    rospy.init_node('client_trial')
    rospy.Subscriber('/exercise/mode', Int32, signal_handler, queue_size=2)
    rospy.Subscriber('/recognizer/output', String, speech_handler, queue_size=1)
    
    start_myo_pub = rospy.Publisher('/myo/launch', Int32, queue_size=1)
    time.sleep(0.5)
    start_myo_pub.publish(Int32(2))

    ## dynamically add publishers to MyoDemo2 class
    MyoDemo2.pub_l = rospy.Publisher('/exercise/l/playback', Quaternion, queue_size=1)
    MyoDemo2.pub_u = rospy.Publisher('/exercise/u/playback', Quaternion, queue_size=1)
    
    print "Classifier launched. Listening to message..."
    rospy.spin()

