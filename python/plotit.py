import matplotlib.pyplot as plt
import numpy as np
from scipy.signal import butter,filtfilt
from ds1054z import DS1054Z
import time

# f = open('data.csv', 'w')


# print p1
# print len(p1)
# exit()


def butter_lowpass_filter(data, cutoff, fs, order):
    normal_cutoff = cutoff / (0.5*fs)
    # Get the filter coefficients 
    b, a = butter(order, normal_cutoff, btype='low', analog=False)
    y = filtfilt(b, a, data)
    return y

a = np.loadtxt('C:\Users\Test-User\Desktop\DS1054Z_screen_capture-master\data2.csv', delimiter=',', skiprows=1)

p1 = a[:,0]
p2 = a[:,1]
p3 = a[:,2]

p1_filt = butter_lowpass_filter(p1, 5e3, 1.25e6, 4)
p2_filt = butter_lowpass_filter(p2, 5e3, 1.25e6, 4)
p3_filt = butter_lowpass_filter(p3, 5e3, 1.25e6, 4)
p3_filt = p3_filt-p2_filt
# N = 10
# p1 = np.convolve(p1, np.ones((N,))/N, mode='same')

x = range(len(p1_filt))
plt.plot(x, p1_filt)
plt.plot(x, p2_filt)
plt.plot(x, p3_filt)
# plt.plot(x, p1)
# plt.plot(x, p2)
# plt.plot(x, p3)

plt.show()