import matplotlib.pyplot as plt
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


print "Average execution time: {}".format( sum(times)/ float( len(times) ) )  
print "Max execution time: {}".format(max(times))
print "Min execution time: {}".format(min(times))

plt.plot(frames,times)
plt.xlabel('Frame')
plt.ylabel('Time [s]')
plt.title('DBoW2 Execution Time')
#plt.legend()
plt.show()

