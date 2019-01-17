import matplotlib.pyplot as plt
import csv
import sys

filename = sys.argv[1]

x = []
y = []

with open(filename,'r') as csvfile:
    plots = csv.reader(csvfile, delimiter=' ')
    for row in plots:
        x.append(int(row[0]))
        y.append(float(row[1]))

plt.plot(x,y)
plt.xlabel('Frame')
plt.ylabel('Time [s]')
plt.title('Visual Odometry Execution Time')
plt.legend()
plt.show()
