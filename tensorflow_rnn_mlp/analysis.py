import sys, os, random, time, math, csv
import matplotlib.pyplot as plt
import numpy as np

truth = {}
predicted = {}
loss = []

if len(sys.argv) is 2:
    run = sys.argv[1];
else:
    run = 1

if not os.path.exists('predictions/' + str(sys.argv[1]) + '/analysis'):
    os.makedirs('predictions/' + str(sys.argv[1]) + '/analysis')

with open('predictions/ground_truth.csv', 'rbb') as f:
    reader = csv.reader(f, delimiter=',')
    for i, line in enumerate(reader):
        if i is 0:
            continue
        instance = line[0]
        truth[instance] = line[2:-1]

with open('predictions/' + str(run) + '/predicted.csv', 'rb') as f2:
    reader = csv.reader(f2, delimiter=',')
    for i, line in enumerate(reader):
        if i is 0:
            continue
        instance = line[0]
        epoch = line[1]
        if epoch is '1':
            predicted[instance] = []
        predicted[instance].append(line[2:-1])

with open('predictions/' + str(run) + '/loss.csv', 'rb') as f2:
    reader = csv.reader(f2, delimiter=',')
    for i, line in enumerate(reader):
        if i is 0:
            continue
        loss.append(float(line[1]))

#Calculate average link count difference
averageLinkCountDifference = []
for epoch in range(len(loss)):
    averageLinkCountDifference.append(0.)
    for instance in predicted:
        averageLinkCountDifference[epoch] = averageLinkCountDifference[epoch] + abs(float(truth[instance][0]) - float(predicted[instance][epoch][0]))
    averageLinkCountDifference[epoch] = averageLinkCountDifference[epoch] / len(truth)

averageLinkDifference = []
for epoch in range(len(loss)):
    averageLinkDifference.append(0.)
    for instance in predicted:
        for link in range(1, len(predicted[instance][epoch])):
             averageLinkDifference[epoch] = averageLinkDifference[epoch] + abs(float(truth[instance][link]) - float(predicted[instance][epoch][link]))
    averageLinkDifference[epoch] = averageLinkDifference[epoch] / (len(truth) * (len(truth[instance]) - 1))

linkBuckets = [[],[],[],[],[],[],[],[]]
for instance in predicted:
    index = int(truth[instance][0]) - 1
    linkBuckets[index].append(float(predicted[instance][len(loss) - 1][0]))

for truth in range(len(linkBuckets)):
    plt.figure()
    plt.title('Distribution of link counts for truth count ' + str(truth + 1))
    plt.hist(linkBuckets[truth], 50)
    plt.savefig('predictions/' + str(run) + '/analysis/count_' + str(truth + 1) + '_histogram')

epochs = range(1, len(loss) + 1)

fig, ax1 = plt.subplots()
ax1.plot(epochs, averageLinkCountDifference, 'b')
ax1.set_xlabel('epochs')
# Make the y-axis label, ticks and tick labels match the line color.
ax1.set_ylabel('Average Difference', color='b')
ax1.tick_params('y', colors='b')

ax2 = ax1.twinx()
ax2.plot(epochs, loss, 'r')
ax2.set_ylabel('Loss', color='r')
ax2.tick_params('y', colors='r')
ax2.yaxis.set_ticks(range(int(min(loss)), int(max(loss)), int((max(loss) - min(loss)) / 10)))
plt.title('Average Difference in Link Count vs loss')
fig.tight_layout()
plt.savefig('predictions/' + str(run) + '/analysis/averageLinkCountDifference.png')


fig, ax1 = plt.subplots()
ax1.plot(epochs, averageLinkDifference, 'b')
ax1.set_xlabel('epochs')
# Make the y-axis label, ticks and tick labels match the line color.
ax1.set_ylabel('Average Difference', color='b')
ax1.tick_params('y', colors='b')

ax2 = ax1.twinx()
ax2.plot(epochs, loss, 'r')
ax2.set_ylabel('Loss', color='r')
ax2.tick_params('y', colors='r')
ax2.yaxis.set_ticks(range(int(min(loss)), int(max(loss)), int((max(loss) - min(loss)) / 10)))
plt.title('Average Difference in Link lengths vs loss')
fig.tight_layout()
plt.savefig('predictions/' + str(run) + '/analysis/averageLinkDifference.png')
