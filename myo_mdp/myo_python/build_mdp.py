'''
Created on Jan 7, 2016

@author: ymeng
'''

import collections
import numpy as np
import mdptoolbox
import cPickle as pickle

SMOOTHING = 0.001

class State(object):
    def __init__(self, imu=None, label=None):
        self.imu = imu
        self.label = label
        self.classifier = None
        if imu:
            self.orientation = imu[6:8]
            
    def getLabel(self):
        if self.classifier is None:
            self.classifier = pickle.load('../data/imu_classifier.pkl')
        if self.imu is not None:
            self.label = self.classifier.predict(self.imu)
        return self.label

class BuildMDP(object):
    '''
    Build transition matrices and reward matrices
    '''
    def __init__(self, actionsData=[], statesData=[], actionsFile=None, statesFile=None, build=True):
        '''
        Constructor
        '''
        def _buildAll():
            self.getTransitions(actionsData, statesData, actionsFile, statesFile)
            self.buildP()
            self.buildR()
            self.findPath()
            self.buildPolicy() 
            
        self.path = []
        self.Pi = {}
        
        if build:
            _buildAll()      
        
       
    
    def getTransitions(self, actionsData, statesData, actionsFile, statesFile):
        
        C3 = collections.defaultdict(int)
        C2 = collections.defaultdict(int)
        self.Pr = {}
        
        if not any(actionsData) and actionsFile:
            with open(actionsFile) as f:
                actionsData = ['a'+x.strip() for x in f]
        #--------------------------------------------------------- print actions
        if not any(statesData) and statesFile:
            with open(statesFile) as f:
                statesData = ['s'+x.strip() for x in f]
        #---------------------------------------------------------- print states
        
        assert len(actionsData) == len(statesData)
        for i in range(len(statesData)):
            #----------------------------------------------------------- print i
            C3[(actionsData[i], statesData[i], statesData[i+1])] += 1
            C2[(actionsData[i], statesData[i])] += 1
            if i == len(statesData)-2: break
        
        for key in C3:
            self.Pr[key] = 1.0*C3[key] / C2[(key[0], key[1])]
        
        self.actions = sorted(list(set(actionsData)))
        self.states = sorted(list(set(statesData)))
        
        # find the starting and ending states
        # check the state labels of the first and last 5 data points
        start = collections.Counter(statesData[0:6])
        self.start_state = start.most_common()[0][0]
        end = collections.Counter(statesData[-6:])
        self.end_state = end.most_common()[0][0]
    
    def getProb(self, a, s, s_next):
        
        if (a, s, s_next) in self.Pr:
            return self.Pr[a, s, s_next]
        else:
            return SMOOTHING
        
    def buildP(self):
        """
        Build the transition probability matrices
        """
            
        n_actions = len(self.actions)
        n_states = len(self.states)
        self.T = np.zeros((n_actions, n_states, n_states))
        
        for i in range(n_actions):
            for j in range(n_states):
                for k in range(n_states):
                    self.T[i, j, k] = self.getProb(self.actions[i], self.states[j], self.states[k])
    
    def buildR(self):
        """
        Build the reward matrices
        """
        n_actions = len(self.actions)
        n_states = len(self.states)
        self.R = np.zeros((n_actions, n_states, n_states))

        for i in range(n_actions):
            for j in range(n_states):
                for k in range(n_states):
                    self.R[i, j, k] = self.getReward(self.actions[i], self.states[j], self.states[k])
        
    
    def findPath(self):
        current_state = self.start_state
        path = []
        while True:
            path.append(current_state)
            if (current_state == self.end_state and len(path)>1) or len(path)>10:
                break
            current_idx = self.states.index(current_state)
            T = self.T[:,current_idx,:]
            # do not consider transition to itself here
            T[:, current_idx] = 0
            next_idx = np.unravel_index(T.argmax(), T.shape)[1]
            current_state = self.states[next_idx]
        self.path = path
           
    def getReward(self, a, s, s_next):
        if s_next == s:
            reward = self.getProb(a, s, s_next)-1
        else:
            # reward = self.getProb(a, s, s_next)
            reward = 0
            for a in self.actions:
                reward += self.getProb(a, s, s_next)
        
        return reward
        
    def buildPolicy(self):
        vi = mdptoolbox.mdp.ValueIteration(self.T, self.R, 0.9, skip_check=True)
        vi.run()
        policy = vi.policy
        for i,item in enumerate(policy):
            self.Pi[self.states[i]] = self.actions[item]           
    
    def findNextState(self, s):
        idx = self.states.index(s)
        T = self.T[:,idx,:]
        T[:, idx] = 0
        next_idx = np.unravel_index(T.argmax(), T.shape)[1]
        next_state = self.states[next_idx] 
        
        return next_state

                
if __name__ == '__main__':
    actionsFile = '../data/emg_labels.dat'
    statesFile = '../data/imu_labels.dat'
    state_map = {'s1':'twisting clockwise', 's2':'reaching limit counter-clockwise', 's3':'starting position', 's4':'twisting counter-clockwise'}
    
    builder = BuildMDP(actionsFile=actionsFile, statesFile=statesFile)
    builder.findPath()
    print "policy:", builder.Pi
    
    print "Demonstrate the policy function:\n"
    for state in builder.path:
        print "current state:     ", state, state_map[state]
        next_state = builder.findNextState(state)
        print "optimal next state:", next_state, state_map[next_state]
        print
 
