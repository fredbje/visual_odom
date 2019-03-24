import matplotlib.pyplot as plt
import numpy as np
import csv
import sys

filename = sys.argv[1]

frames = []
times = []

with open(filename,'r') as csvfile:
    plots = csv.reader(csvfile, delimiter=' ')
    for row in plots:
        frames.append(int(row[0]))
        times.append(float(row[1]))

print "Max: ", np.max(times)
print "Min: ", np.min(times)
print "Mean: ", np.mean(times)
print "Std: ", np.std(times)

plt.plot(frames,times)
plt.xlabel('Frame')
plt.ylabel('Time [s]')
plt.title('Developed System')
plt.legend()
plt.show()
