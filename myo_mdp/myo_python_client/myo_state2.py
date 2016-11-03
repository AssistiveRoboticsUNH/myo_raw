'''
Created on Mar 25, 2016

@author: ymeng
'''

import cPickle as pickle
import rospy
from std_msgs.msg import String, Float64, Empty, Int32
from geometry_msgs.msg import Quaternion
from myo_raw.msg import IMUData, EMGIMU
import numpy as np
from myo_demo2 import MyoDemo2
import time
import preprocess
from align_signal import dtw_distance, align_signal
from scipy.signal import savgol_filter # need scipy version 0.16
import threading
import sys
import math
from collections import defaultdict
import matplotlib.pyplot as plt
import datetime

TIME_WEIGHT = 0.05
EMG_WEIGHT = 1
END = -1


class MyoPrompt2(MyoDemo2):
    def __init__(self, task_type=1):
        super(MyoPrompt2, self).__init__(task_type=task_type)
        self.skip = False
        
    def callback(self, percentage, end=None):
        # print 'prompting range: ', percentage, end
        counter = 0
        starting = int(percentage*len(self.quat_l))
        if end is not None:
            ending = int(end*len(self.quat_l))
        else:
            ending = min(starting+50, len(self.quat_l)-1)

        rate = rospy.Rate(20)
        for i in range(starting, ending):
            if self.skip:
                print "Demonstration skipped"
                self.skip == False
                MyoDemo2.pub_l.publish(Quaternion(x=-1337, y=-1337, z=-1337, w=-1337))
                MyoDemo2.pub_u.publish(Quaternion(x=-1337, y=-1337, z=-1337, w=-1337))
                return
            MyoDemo2.pub_l.publish(self.quat_l[i])
            MyoDemo2.pub_u.publish(self.quat_u[i])
            counter += 1
            rate.sleep()
        if end != 1:
            MyoDemo2.pub_l.publish(Quaternion(x=-1337, y=-1337, z=-1337, w=-1337))
            MyoDemo2.pub_u.publish(Quaternion(x=-1337, y=-1337, z=-1337, w=-1337))
        print counter, "samples pulished."

    def subscriber(self):
        """Do not use subscriber here. 
           The callback function will be called directly without ros message
        """
        pass


class MyoState(object):
    def __init__(self, input_list, max_n):

        self.count_ngrams(input_list+[END], max_n)
        self.build_predictor()

    def get_ngrams(self, input_list, n):
        return zip(*[input_list[i:] for i in range(n)])

    def count_ngrams(self, input_list, max_n=3):
        self.ngram_counter = defaultdict(int)
        max_n = min(len(input_list), max_n)
        for n in range(1, max_n+1):
            ngrams = self.get_ngrams(input_list, n)
            for item in ngrams:
                self.ngram_counter[item] += 1
        return self.ngram_counter

    def build_predictor(self):
        self.predictor = {}
        for ngram in self.ngram_counter:
            if len(ngram) > 1:
                prediction = ngram[-1]
                condition = ngram[:-1]
                prob = 1.0*self.ngram_counter[ngram] / self.ngram_counter[condition]
                if condition not in self.predictor:
                    self.predictor[condition] = (prob, prediction)
                elif prob > self.predictor[condition][0]:
                    self.predictor[condition] = (prob, prediction)

    def next_step(self, history):
        while history and (tuple(history) not in self.predictor):
            history = history[1:]
        if history:
            return self.predictor[tuple(history)][1]
        else:
            return None

def evaluate(emg_labels, state_labels, mdp_builder):
    """Baseline of performance
    """
    total_reward = 0
    N = len(emg_labels)
    print "Number of points: ", N
    for i in range(N - 1):
        s = state_labels[i]
        a = emg_labels[i]
        s_next = state_labels[i + 1]
        total_reward += mdp_builder.getReward(a, s, s_next)

    return total_reward


class Evaluator(object):
    def __init__(self, task_type):
        # self.segments = pickle.load(open('../data/segments.dat'))
        self.emg_u = np.genfromtxt('../data'+str(task_type)+'/emg_u.dat', delimiter=',')
        self.emg_l = np.genfromtxt('../data'+str(task_type)+'/emg_l.dat', delimiter=',')
        self.ort_u = np.genfromtxt('../data'+str(task_type)+'/demo_u.dat', delimiter=',')
        self.ort_l = np.genfromtxt('../data'+str(task_type)+'/demo_l.dat', delimiter=',')

class Progress(object):
    
    def __init__(self, classifier=None, classifier_pkl='../data/state_classifier.pkl', give_prompt=True, **kwargs):

        self.logfile = '../log/evaluation.log'
        if classifier:
            self.classifier = classifier
            self.EMG_MAX = kwargs.get('EMG_MAX', 585)
            self.EMG_MIN = kwargs.get('EMG_MIN', 0)
            self.GYRO_MAX = kwargs.get('GYRO_MAX', 500)
            self.baseline = kwargs.get('baseline', 0)
        elif classifier_pkl:
            self.classifier, self.EMG_MAX, self.EMG_MIN, self.GYRO_MAX, self.baseline = pickle.load(open(classifier_pkl))
        self.task_type = kwargs.get('task_type', '')
        self.getTask()
        self.reset()

        self.user_id = kwargs.get('id', 'new_user')
        self.mdp = kwargs.get('mdp', None)
        self.pub = rospy.Publisher('/exercise/progress', Float64, queue_size=10)
        self.pub1 = rospy.Publisher('/exercise/state', String, queue_size=10)
        self.score_pub = rospy.Publisher('/exercise/score', Int32, queue_size=1)
        try:
            threading.Thread(target=self.activatePrompt).start()
            #self.activatePrompt()
        except:
            print "Could not start thread. ", sys.exc_info()[0]
            sys.exit(1)
        with open(self.logfile, 'ab+') as f:
            f.write('\n{:%Y-%m-%d %H:%M:%S} task started.\n'.format(datetime.datetime.now()))
            f.write('task type: %s\n' %self.task_type)
        self.finished = False
        self.subscribeIMU() # this will trigger getProgress

    def reset(self):
        self.n_prompts = 0
        self.emgimu_l = None
        self.emgimu_u = None
        self.task = self.full_task[:]  # copy the list
        print "task: ", self.task
        self.history = []
        self.state_history = []
        self.current_segment = []
        self.state_tracer = []
        self.recent_state = None
        self.progress = 0.0
        self.delay = 0  # time to give prompt if no progress
        self.prompt_now = False
        self.previous = []
        #self.prompt = None
        with open(self.logfile, 'ab+') as f:
            f.write('\n{:%Y-%m-%d %H:%M:%S} task reset.\n'.format(datetime.datetime.now()))

    def activatePrompt(self):
        self.prompt = MyoPrompt2(task_type=self.task_type)

    def getTask(self):
        with open('../data'+str(self.task_type)+'/state_sequence.dat') as f:
            self.full_task = [int(x.strip()) for x in f]
        self.n_states = len(self.full_task)
        #self.task.pop(0) # do not require start state
        self.myo_state = MyoState(self.full_task, 7)
        print self.myo_state.predictor
        time.sleep(1)
    
    def callback(self, emgimu):
        if self.finished:
            return
        if self.emgimu_l is None:
            return
        self.updateUpper(emgimu)
        if not self.prompt_now and not self.finished:
            self.getProgress(self.emgimu_l, self.emgimu_u)
    
    def updateLower(self, emgimu):
        if self.finished:
            return
        self.emgimu_l = emgimu
        
    def updateUpper(self, emgimu):
        self.emgimu_u = emgimu
    
    def subscribeIMU(self):
        rospy.Subscriber('/myo/l/emgimu', EMGIMU, self.updateLower, queue_size=1)
        rospy.Subscriber('/myo/u/emgimu', EMGIMU, self.callback, queue_size=1)
        rospy.Subscriber('/exercise/detected_state', String, self.speech_handler, queue_size=1)

    def getPosition(self, state):
        next_state = self.myo_state.next_step(self.state_history)
        if next_state == END:
            return self.n_states-1
        position = 0
        for i, item in enumerate(self.full_task):
            if i == len(self.full_task)-1:
                break
            if item == state and self.full_task[i+1] == next_state:
                # position = i
                return i
        return position

    def getProgress(self, emgimu_l, emgimu_u):
        #print "Tracking your progress now..."

        emg_l = preprocess.process_emg(np.array(emgimu_l.emg), self.EMG_MAX, self.EMG_MIN)
        acc_l = np.array(emgimu_l.linear_acceleration)
        gyro_l = preprocess.process_gyro(np.array(emgimu_l.angular_velocity), self.GYRO_MAX, discrete=False)
        orie_l = np.array(emgimu_l.orientation)

        emg_u = preprocess.process_emg(np.array(emgimu_u.emg), self.EMG_MAX, self.EMG_MIN)
        acc_u = np.array(emgimu_u.linear_acceleration)
        gyro_u = preprocess.process_gyro(np.array(emgimu_u.angular_velocity), self.GYRO_MAX, discrete=False)
        orie_u = np.array(emgimu_u.orientation)
        
        signal_array = np.hstack((EMG_WEIGHT*emg_l, acc_l, gyro_l, orie_l, EMG_WEIGHT*emg_u, acc_u, gyro_u, orie_u))
        self.history.append(signal_array)
        # print "# data points saved: ", len(self.history)
        if len(self.previous)>10: # 0.2 seconds
            self.previous.pop(0)
        self.previous.append(signal_array)
        current_signal = np.mean(self.previous, axis=0)

        state = int(self.classifier.predict(current_signal)[0])
        self.state_tracer.append(state)
        self.pub1.publish(str(state))

        if len(self.state_tracer) < 5:
            return

        if state != -1 and state == self.state_tracer[-2] and state == self.state_tracer[-3]:
            if not self.state_history:
                self.state_history.append(state)
            elif state != self.state_history[-1]:
                self.state_history.append(state)
                print "state history: ", self.state_history

            self.current_segment.append(len(self.history))
            self.progress = self.getPosition(state) * 1.0 / (self.n_states - 1)
            #print "progress: ", self.progress

            # print "current state", state, state_map[state]
            if state in self.task:
                self.task.remove(state)
                #self.progress = 1 - 1.0*len(self.task)/(self.n_states-1)
            if state != self.recent_state:
                self.delay = 0
            self.recent_state = state
            self.delay += 1

    def deliver_prompt(self):
        print "Prompt started..."
        print self.progress
        self.prompt_now = True

        if self.progress >= 0:
            self.prompt.callback(self.progress)
        elif self.progress == 1:
            print "No need to prompot"
        else:
            print "do not know how to prompt..."
        print "prompt ended"
        self.delay = 0
        self.prompt_now = False
        self.n_prompts += 1

    def end_game(self):
        self.finished = True
        with open(self.logfile, 'a') as f:
            f.write('{:%Y-%m-%d %H:%M:%S} task completed. '.format(datetime.datetime.now()))
        #self.pub.publish(1.0)
        try:
            self.evaluate_pfmce()
        except IndexError:
            print "Not enough data points to perform evaluation!"
        finally:
            self.reset()

    def speech_handler(self, msg):
        if msg.data == 'help':
            print "Prompt requested by user"
            self.deliver_prompt()

        if msg.data == 'skip': # to stop helping
            self.prompt.skip = True

    def evaluate_pfmce(self, evaluate_emg=True):
        print "Evaluating performance...."
        # if not self.mdp:
        #     print "No MDP found"
        #     self.history = []
        #     return

        #action_classifier = pickle.load(open('../data/action_classifier.pkl'))
        history = np.array(self.history)
        print "raw history shape:", history.shape
        history = history[5:-6:5, :]  # downsampling, cut the first half second
        print "saved history: ", history.shape
        if history.shape[0]> 35:
            history = savgol_filter(history, 31, 3, axis=0) # smoothing

        np.savetxt('user_data/' + self.user_id, history, delimiter=',')

        evaluator = Evaluator(self.task_type)
        emg_l = history[:,0:8] / EMG_WEIGHT
        emg_u = history[:,18:26] / EMG_WEIGHT
        ort_l = history[:,14:18]
        ort_u = history[:,-4:]

        # plt.figure()
        # plt.plot(evaluator.ort_l)
        # plt.figure()
        # plt.plot(align_signal(evaluator.ort_l, ort_l, has_time=False))
        # plt.show(block=True)

        (diff_emg_l, cost) = dtw_distance(evaluator.emg_l, emg_l)
        (diff_emg_u, _) = dtw_distance(evaluator.emg_u, emg_u)
        (diff_ort_l, _) = dtw_distance(evaluator.ort_l, ort_l)
        (diff_ort_u, _) = dtw_distance(evaluator.ort_u, ort_u)
        print "difference cost, alignment cost"
        print (diff_emg_l, cost)
        print diff_emg_u
        print diff_ort_l
        print diff_ort_u
        if evaluate_emg:
            EMG_FACTOR = 0.08
        else:
            EMG_FACTOR = 0
        ORT_FACTOR = 0.25
        TIME_FACTOR = 0.05

        performance = 100 * math.exp(-EMG_FACTOR*(diff_emg_l+diff_emg_u) \
                                     - ORT_FACTOR*(diff_ort_l+diff_ort_u) \
                                     - TIME_FACTOR*cost) - 8*self.n_prompts
        print performance
        self.score_pub.publish(int(performance))
        with open(self.logfile, 'a') as f:
            f.write('performance score: %f\n' %performance)

if __name__ == '__main__':
    progress = Progress(give_prompt=True)

    
