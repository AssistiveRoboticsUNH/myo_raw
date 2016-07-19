'''
Created on Jan 25, 2016

@author: ymeng
'''
from dtwtoolbox.dtw import dtw
from sklearn.metrics.pairwise import euclidean_distances
import numpy as np

def dtw_distance(s, t):
    """
    every column of s or t is a time series
    every row is dimensions of signal at one time point
    w size is symmetric. w=5 means the window has size 11.
    """

    (new_t, diff) = align_signal(s, t, has_time=False, get_distance=True)
    cost = 1.0*(len(t)-len(s))/len(s)

    return (diff, cost)


def align_signal(s, t, w=5, has_time=True, get_distance=False):
    """
    every column of s or t is a time series
    every row is dimensions of signal at one time point
    w size is symmetric. w=5 means the window has size 11.
    """
    
    if has_time:
        s = s[:, 1:]
        t = t[:, 1:]
        
    dist_fun = euclidean_distances
    dist, cost, acc, path = dtw(s, t, dist_fun)
    path = np.array(path)
    
    warped_t = t[path[1, :], :]
    new_t = np.zeros(s.shape)
    
    for i in range(warped_t.shape[0]):
        new_t[path[0, i], :] = warped_t[i, :]
    
    if has_time:
        Ts = np.arange(1, s.shape[0]+1)
        Ts = Ts.reshape(-1, 1)
        new_t = np.hstack((Ts, new_t))
    if get_distance:
        return (new_t, dist)
    return new_t

if __name__ == '__main__':
    import matplotlib.pyplot as plt
    
    x = np.array([[0, 0], [0, 1], [1, 1], [1, 2], [2, 2], [4, 3], [2, 3], [1, 1], [2, 2], [0, 1]])
    y = np.array([[1, 0], [1, 1], [1, 1], [2, 1], [4, 3], [4, 3], [2, 3], [3, 1], [1, 2], [1, 0]])
    dist_fun = euclidean_distances
    new_y = align_signal(x, y, dist_fun, has_time=False)
    plt.plot(x[:,0])
    plt.plot(new_y[:,0])
    plt.plot(y[:,0])
    plt.show()