'''
Created on Jan 11, 2016

@author: ymeng
'''

from sklearn.neighbors import KNeighborsClassifier
from sklearn.cluster import KMeans
import numpy as np
import cPickle as pickle
from sklearn import metrics
from align_signal import dtw_distance


class SignalCluster(object):
    def __init__(self, X, n_clusters=4):
        self.km_model = KMeans(n_clusters=n_clusters).fit(X)
        self.labels = self.km_model.labels_
            
    def evaluate(self, X, labels=None):
        if labels is None:
            labels = self.labels
        return metrics.silhouette_score(X, labels, metric='euclidean')
        
    def save_labels(self, filename):
        np.savetxt(filename, self.labels, fmt='%d')
        

class SignalClassifier(KNeighborsClassifier):
    '''
    classdocs
    '''

    def __init__(self, n_neighbors=5, dim=10, use_dtw=False):
        '''
        Constructor
        '''
        self.dim = dim
        if use_dtw:
            super(SignalClassifier, self).__init__(n_neighbors=n_neighbors, metric='pyfunc', func=dtw_distance)
        else:
            super(SignalClassifier, self).__init__(n_neighbors=n_neighbors)
    
    def train(self, X, y, trainingFile='../data/imu_classifier.pkl'):
        if trainingFile:
            data = np.genfromtxt(trainingFile, delimiter=',')
            X = data[:, 0:self.dim]
            y = data[:, self.dim]
        self.fit(X, y)
    
    def classify(self, X):
        self.predict(X) 

        
if __name__ == '__main__':
    imu_data = '../data/imu.dat'
    imu_classifier = SignalClassifier()
    imu_classifier.train(imu_data, 10)
    print imu_classifier.__dict__
    with open('../data/imu_classifier.pkl', 'wb') as f:
        pickle.dump(imu_classifier, f)
