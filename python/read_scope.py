import matplotlib.pyplot as plt
import numpy as np
from scipy.signal import butter,filtfilt
from ds1054z import DS1054Z
import time

scope = DS1054Z('192.168.0.18')
print(scope.idn)
scope.memory_depth = 300e3

time.sleep(0.5)
p1 = scope.get_waveform_samples(1, mode='RAW')
print len(p1)
p2 = scope.get_waveform_samples(2, mode='RAW')
print len(p2)
p3 = scope.get_waveform_samples(3, mode='RAW')
print len(p3)

f = open('C:\Users\Test-User\Desktop\DS1054Z_screen_capture-master\data2.csv', 'w')

for p,q,r in zip(p1,p2,p3):
    f.write("{},{},{}\n".format(p,q,r))