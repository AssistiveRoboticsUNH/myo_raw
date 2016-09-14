#!/usr/bin/env python

'''
Created on Mar 24, 2016

@author: ymeng
'''
import matplotlib.pyplot as plt
from preprocess import preprocess
import classifier
import myo_state2
from myo_demo2 import MyoDemo2
from build_mdp import BuildMDP
import cPickle as pickle
import argparse
import os
import numpy as np
from align_signal import align_signal
import subprocess
import glob
import time
from geometry_msgs.msg import Quaternion
#from logger import MyLogger


TIME_WEIGHT = 0.05 # weight of temporal coordinates in classification
EMG_WEIGHT = 1
#logger = MyLogger().logger


def build_classifier(samples=1, nClusters=None, ID='user', used=False, has_matlab=False):
    if used:
        mdp = pickle.load(open('../data/mdp.pkl'))
        args = {"give_prompt": True,
                "mdp": mdp,
                "id": ID
                } 
        
        progress = myo_state2.Progress(classifier_pkl='../data/state_classifier.pkl', **args)
        return
    
    if has_matlab is None:
        has_matlab = os.path.exists('/usr/local/MATLAB')

    for identifier in ('l', 'u'):
        for i in range(samples):
            data_i, EMG_max_i, EMG_min_i, GYRO_max_i, states_i = preprocess(os.path.join('../../data/work/', str(i)), update_extremes=True, identifier=identifier)
            emg_i = data_i[:, 1:9]
            imu_i = data_i[:, 9:]
            if i == 0:
                emg_demos = emg_i # use the first emg sample as demo
                imu_demos = imu_i
                emg_data = emg_i
                imu_data = imu_i
                state_labels = states_i
                EMG_MAX = EMG_max_i
                EMG_MIN = EMG_min_i
                GYRO_MAX = GYRO_max_i
                Time_a = np.arange(imu_i.shape[0]).reshape((imu_i.shape[0],1))

                ort_data_a = np.hstack((Time_a, imu_i[:,-4:]))
                emg_data_a = np.hstack((Time_a, emg_i))

            else:
                imu_i_a = align_signal(imu_demos/i, imu_i, w=5, has_time=False)
                imu_demos += imu_i_a
                emg_data = np.vstack((emg_data, emg_i)) # concatenate emg signals
                imu_data = np.vstack((imu_data, imu_i))
                EMG_MAX += EMG_max_i
                EMG_MIN += EMG_min_i
                GYRO_MAX += GYRO_max_i
                
                state_labels = np.concatenate((state_labels, states_i)) # all labels
                emg_i_a = align_signal(emg_demos, emg_i, w=5, has_time=False)

                # Used by Matlab function to compute the expected trojectory
                # The signals are aligned to the first one
                # Potentional improvement: align the to the one with medium length.
                ort_data_a = np.vstack( (ort_data_a, np.hstack((Time_a, imu_i_a[:,-4:]))) )

                emg_data_a = np.vstack( (emg_data_a, np.hstack((Time_a, emg_i_a))) )
            
        n = len(imu_demos)
        
        # get average
        if identifier == 'l':
            print "\nProcessing myo on lower arm..."
            emg_demos_l = emg_demos                        
            imu_demos_l = imu_demos/samples
            EMG_MAX_l = EMG_MAX/samples
            EMG_MIN_l = EMG_MIN/samples
            GYRO_MAX_l = GYRO_MAX/samples
            emg_data_l = emg_data
            emg_data_l_timed = emg_data_a  # timed, for GMM
            imu_data_l = imu_data
            ort_data_l = ort_data_a
            print "EMG_MAX_l", EMG_MAX_l
            print "EMG_MIN_l", EMG_MIN_l
            print "GYRO_MAX_l", GYRO_MAX_l
            print "# data points", len(imu_demos_l)
            
        else:
            print "\nProcessing myo on upper arm..."
            emg_demos_u = emg_demos 
            imu_demos_u = imu_demos/samples
            EMG_MAX_u = EMG_MAX/samples
            EMG_MIN_u = EMG_MIN/samples
            GYRO_MAX_u = GYRO_MAX/samples
            emg_data_u = emg_data
            emg_data_u_timed = emg_data_a
            imu_data_u = imu_data
            ort_data_u = ort_data_a
            print "EMG_MAX_u", EMG_MAX_u
            print "EMG_MIN_u", EMG_MIN_u
            print "GYRO_MAX_u", GYRO_MAX_u
            print "# data points", len(imu_demos_u)
            
    N_l = imu_demos_l.shape[0]
    N_u = imu_demos_u.shape[0]
    N_min = min(N_l, N_u)
    print N_min
    emg_demos_u = emg_demos_u[0:N_min, :]
    imu_demos_u = imu_demos_u[0:N_min, :]
    emg_demos_l = emg_demos_l[0:N_min, :]
    imu_demos_l = imu_demos_l[0:N_min, :]
    Time = np.arange(N_min).reshape((N_min, 1))

    show_ort_l = np.genfromtxt('../../data/work/0/ort_l.mat', delimiter=',')
    show_ort_l = show_ort_l[16:-6:2,:]
    np.savetxt('../data/prompt_l.dat', show_ort_l, delimiter=',')
    show_ort_u = np.genfromtxt('../../data/work/0/ort_u.mat', delimiter=',')
    show_ort_u = show_ort_u[16:-6:2, :]
    np.savetxt('../data/prompt_u.dat', show_ort_u, delimiter=',')

    if has_matlab:
        print "MATLAB FOUND"
        ## use the following if there is Malab under /usr/local
        ## GMM and GMR will be performed
        os.chdir('../matlab')
        np.savetxt('ort_data_u', ort_data_u, delimiter=',')
        np.savetxt('ort_data_l', ort_data_l, delimiter=',')
        np.savetxt('emg_data_u', emg_data_u_timed, delimiter=',')
        np.savetxt('emg_data_l', emg_data_l_timed, delimiter=',')
        subprocess.call(['matlab', '-nodisplay', '-nojvm', '-nosplash', '-r', 'demo_gmm();exit'])
        os.chdir('../myo_python')
    else:
        print "MATLAB NOT FOUND"
        ## use the following if there is no Matlab
        ## simply use the aligned average as expected trajectory
        np.savetxt('../data/demo_l.dat', imu_demos_l[:, -4:], delimiter=',')
        np.savetxt('../data/demo_u.dat', imu_demos_u[:, -4:], delimiter=',')
        np.savetxt('../data/emg_l.dat', emg_demos_l, delimiter=',')
        np.savetxt('../data/emg_u.dat', emg_demos_u, delimiter=',')

    emg_demos = np.hstack((emg_demos_l,emg_demos_u))
    emg_cluster = classifier.SignalCluster(emg_demos, n_clusters=8)
    
    observations = np.hstack((EMG_WEIGHT*emg_demos_l, imu_demos_l, EMG_WEIGHT*emg_demos_u, imu_demos_u))

    if nClusters is None:
        N = max(state_labels)  # number of tagged points
        print "state labels:", N
        low_limit = max(2, N / 2) # at least 2 clusters
        high_limit = N
        scores = {}
        for n in range(low_limit, high_limit+2):
            state_cluster = classifier.SignalCluster(observations, n)
            score = state_cluster.evaluate(observations, state_cluster.labels)
            scores[score] = n
            print '# clusters, score', n, score

        max_score = max(scores.keys())
        n_clusters = scores[max_score]
        state_cluster = classifier.SignalCluster(observations, n_clusters)
    else:
        state_cluster = classifier.SignalCluster(observations, nClusters)

    plt.figure()
    plt.plot(show_ort_l)
    #plt.plot(emg_demos_u)
    plt.plot(state_cluster.labels, '*')
    plt.show(block=True)

    ## Currently we implement the n-gram model without explicitly training the MDP
    ## So the following code is not used
    ## ============================================================================
    ## build mdp
    #
    # indexes = [x for x,item in enumerate(valid_states) if item] # critical points
    # segments = []
    # left = indexes[0]
    # for i,ind in enumerate(indexes):
    #     if ind-left > 5:
    #         segments.append((left, ind))
    #         left = ind
    # print segments
    # pickle.dump(segments, open('../data/segments.dat', 'w'))

    #actionsData = emg_cluster.labels[valid_states]
    #statesData = statesData[valid_states]

    # builder = BuildMDP(actionsData=statesData, statesData=statesData) # use the same labels
    # pickle.dump(builder, open('../data/mdp.pkl', 'wb'))
    #
    # print "path", builder.path
    # print "policy", builder.Pi

    #print "expected actions: ", actionsData
    ## ==============================================================================

    print "expected states: ", state_cluster.labels

    #baseline = evaluate(actionsData, statesData, builder)
    #print "baseline performance: ", baseline

    state_classifier = classifier.SignalClassifier(n_neighbors=5)
    state_classifier.train(observations, state_cluster.labels, trainingFile=None)
    baseline = None
    pickle.dump((state_classifier, EMG_MAX, EMG_MIN, GYRO_MAX, baseline), open('../data/state_classifier.pkl', 'wb'))

    state_labels = state_labels[0:N_min]
    critical_points = observations[state_labels>=0]
    task = state_classifier.predict(critical_points)
    state_sequence = []
    with open('../data/state_sequence.dat', 'w') as f:
        for item in task:
            if item not in state_sequence or item != state_sequence[-1]:
                state_sequence.append(item)
                f.write(str(item)+'\n')

    #action_classifier = classifier.SignalClassifier(n_neighbors=5)
    #action_classifier.train(emg_demos, emg_cluster.labels, trainingFile=None)
    #pickle.dump(action_classifier, open('../data/action_classifier.pkl', 'wb'))

def gather_samples_and_build():
    makeProcess = subprocess.Popen(['python', 'makeData2.py', '../../data/work']) 
    makeProcess.wait()

    bagFiles = glob.glob(os.path.join('../../data/work', '*.bag'))
    samples = len(bagFiles)
    print "number of training samples:", samples
    if samples == 0:
        "No data to process!"
        return
    build_classifier(samples=samples)

progress = None
is_running = False

def signal_handler(msg):
    #logger.info('signal received...')
    print "signal received", msg

    global progress
    global is_running

    if msg.data != 1 and progress is not None:
        if is_running:
            is_running = False
        progress.end_game()
        progress = None

    if msg.data == 0:
        # There is intentional repetition of this line in both conidition statements rather than
        # at the top of the routine because msg.data is not always 0 or 1
        is_running = False
        gather_samples_and_build() 
        demo = myo_state2.MyoPrompt2()
        time.sleep(.5)
        demo.callback(0, 1)

    elif msg.data == 1: # practice
        #gather_samples_and_build() 
        mdp = pickle.load(open('../data/mdp.pkl'))
        args = {"give_prompt": True,
                "mdp": mdp,
                "id": "new patient"
                } 
        if progress is None:
            progress = myo_state2.Progress(classifier_pkl='../data/state_classifier.pkl', **args)
            is_running = True
        else:
            if is_running:
                progress.reset()

            progress = myo_state2.Progress(classifier_pkl='../data/state_classifier.pkl', **args)
            is_running = True

if __name__ == '__main__':
    import rospy
    from std_msgs.msg import Int32, Empty

    #logger.info('Classifier launched. Listening to message...')

    print "Running developer mode..."
    rospy.init_node('build_classifier')
    #logger.info('after node...')
    rospy.Subscriber('/exercise/mode', Int32, signal_handler)

    MyoDemo2.pub_l = rospy.Publisher('/exercise/l/playback', Quaternion, queue_size=1)
    MyoDemo2.pub_u = rospy.Publisher('/exercise/u/playback', Quaternion, queue_size=1)

    print "Classifier launched. Listening to message..."

    rospy.spin()

