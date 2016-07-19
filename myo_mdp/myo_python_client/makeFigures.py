# -*- coding: utf-8 -*-
import matplotlib.pyplot as plt
import numpy as np
import os
from scipy.signal import savgol_filter
from align_signal import align_signal

dataPath0='/home/ymeng/jade/myo/src/myo_raw/data/work/0'
quat0 = np.genfromtxt(os.path.join(dataPath0,'gyro.mat'), delimiter=',')
QUAT0 = quat0[6:-6:5, :]
QUAT0_smooth = savgol_filter(QUAT0, 31, 3, axis=0)

dataPath1='/home/ymeng/jade/myo/src/myo_raw/data/work/1'
quat1 = np.genfromtxt(os.path.join(dataPath1,'gyro.mat'), delimiter=',')
QUAT1 = quat1[6:-6:5, :]
QUAT1_smooth = savgol_filter(QUAT1, 31, 3, axis=0)

fig = plt.figure()
fig.suptitle('Signal 1 & 2 Overlaid with DTW', fontsize=18, fontweight='bold')
ax = fig.add_subplot(111)
fig.subplots_adjust(top=0.85)
#ax.set_title('axes title')
ax.set_xlabel('samples', fontsize=16)
ax.set_ylabel('ampitude', fontsize=16)
signal0 = np.reshape(QUAT0_smooth[:,0], (-1,1))
ax.plot(signal0)

#fig1 = plt.figure()
#fig1.suptitle('Signal 2', fontsize=18, fontweight='bold')
#ax1 = fig1.add_subplot(111)
#fig1.subplots_adjust(top=0.85)
##ax.set_title('axes title')
#ax1.set_xlabel('samples', fontsize=16)
#ax1.set_ylabel('ampitude', fontsize=16)
signal1 = np.reshape(QUAT1_smooth[:,0], (-1,1))
ax.plot(signal1, 'r')


aligned = align_signal(signal0, signal1, w=5, has_time=False);
ax.plot(aligned, 'r--')

plt.show()
