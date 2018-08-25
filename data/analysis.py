import sys, os, random, time, math, csv
import matplotlib.pyplot as plt
import numpy as np

START_EPOCH = 1
MAX_EPOCH = 100
MAX_ACCURACY_EPOCH = 70
START_LINK = 2
MAX_LINK = 6
COLORS = ['b', 'r', 'g', 'c', 'm', 'y']
EPOCHS = range(START_EPOCH, MAX_EPOCH + 1)

class LineData:
    def __init__(self, xAxisData, yAxisData, lineColor='k', lineLabel='line', lineStyle='-'):
        self.xData = xAxisData
        self.yData = yAxisData
        self.color = lineColor
        self.label = lineLabel
        self.style = lineStyle

class AnalysisPlot:
    def __init__(self, lineData, xAxisLabel='X', yAxisLabel='Y', plotTitle='Plot', savePlot=False, saveName='graph.png', savePath='', showPlot=True):
        self.line = lineData
        self.xlabel = xAxisLabel
        self.ylabel = yAxisLabel
        self.title = plotTitle
        self.show = showPlot
        self.save = savePlot
        self.path = savePath
        self.filename = saveName

    def plot(self):
        plt.figure()
        for i in range(len(self.line)):
            plt.plot(self.line[i].xData, self.line[i].yData, self.line[i].color, label=self.line[i].label, linestyle=self.line[i].style)
        plt.xlabel(self.xlabel)
        plt.ylabel(self.ylabel)
        plt.title(self.title)
        plt.legend()
        if self.show is True:
            plt.show()
        if self.save is True:
            plt.savefig(self.path + self.filename)

def fetchData(link):
    data = { 'truth': {}, 'predict': {}, 'loss': [] }

    if link is -1:
        folder = 'one_hot_pred_non_occluded'
    else:
        folder = 'pipe_' + str(link) + '_non_occluded'

    with open(folder + '/ground_truth.csv', 'rb') as f:
        reader = csv.reader(f, delimiter=',')
        for i, line in enumerate(reader):
            if i is 0:
                continue;
            instance = line[0]
            data['truth'][instance] = line[2:]

    with open(folder + '/predicted.csv', 'rb') as f2:
        reader = csv.reader(f2, delimiter=',')
        for i, line in enumerate(reader):
            if i is 0:
                continue
            instance = line[0]
            epoch = int(line[1])
            if epoch < START_EPOCH or epoch > MAX_EPOCH:
                continue
            if epoch is START_EPOCH:
                data['predict'][instance] = []
            data['predict'][instance].append(line[2:])

    with open(folder + '/loss.csv', 'rb') as f2:
        reader = csv.reader(f2, delimiter=',')
        for i, line in enumerate(reader):
            if i < START_EPOCH or i > MAX_EPOCH:
                continue
            data['loss'].append(float(line[1]))
    return data

def averageLinkDifference(data):
    avg = []
    for expectedLinks in range(len(data)):
        avg.append([])
        for epoch in range(len(data[expectedLinks]['loss'])):
            avg[expectedLinks].append(0.)
            for instance in data[expectedLinks]['predict']:
                for link in range(1, len(data[expectedLinks]['predict'][instance][epoch])):
                    avg[expectedLinks][epoch] = avg[expectedLinks][epoch] + abs(float(data[expectedLinks]['truth'][instance][link]) - float(data[expectedLinks]['predict'][instance][epoch][link]))
            avg[expectedLinks][epoch] = avg[expectedLinks][epoch] / (len(data[expectedLinks]['truth']) * (len(data[expectedLinks]['truth'][instance]) - 1))
    return avg

def totalLinkDifference(data):
    total = []
    for expectedLinks in range(len(data)):
        total.append([])
        for epoch in range(len(data[expectedLinks]['loss'])):
            total[expectedLinks].append(0.)
            for instance in data[expectedLinks]['predict']:
                gtTotal = 0
                predictTotal = 0
                for link in range(1, len(data[expectedLinks]['predict'][instance][epoch])):
                    gtTotal = gtTotal + float(data[expectedLinks]['truth'][instance][link])
                    predictTotal = predictTotal + float(data[expectedLinks]['predict'][instance][epoch][link])
                total[expectedLinks][epoch] = total[expectedLinks][epoch] + abs(float(gtTotal) - float(predictTotal))
            total[expectedLinks][epoch] = total[expectedLinks][epoch] / len(data[expectedLinks]['truth'])
    return total

def linkCounts(data):
    count = { 'truth' : [0,0,0,0,0], 'predict' : [] , 'accuracy' : [[], [], [], [], []] , 'totalAccuracy' : [] }
    # count truth
    for instance in data['truth']:
        for link in range(len(data['truth'][instance])):
            value = int(data['truth'][instance][link])
            if value is 1:
                count['truth'][link] = count['truth'][link] + 1

    for epoch in range(len(data['predict']['arm_304'])):
        count['predict'].append([0,0,0,0,0])
        count['accuracy'][0].append(0.)
        count['accuracy'][1].append(0.)
        count['accuracy'][2].append(0.)
        count['accuracy'][3].append(0.)
        count['accuracy'][4].append(0.)
        count['totalAccuracy'].append(0.)

        for instance in data['predict']:
            for link in range(len(data['predict'][instance][epoch])):
                value = int(data['predict'][instance][epoch][link])
                if value is 1:
                    count['predict'][epoch][link] = count['predict'][epoch][link] + 1
                    truthValue = int(data['truth'][instance][link])
                    if value is truthValue:
                        count['accuracy'][link][epoch] = count['accuracy'][link][epoch] + 1
                        count['totalAccuracy'][epoch] = count['totalAccuracy'][epoch] + 1

    for epoch in range(len(count['accuracy'][0])):
        for link in range(len(count['accuracy'])):
            count['accuracy'][link][epoch] = float(count['accuracy'][link][epoch]) / float(count['truth'][link])
        count['totalAccuracy'][epoch] = float(count['totalAccuracy'][epoch]) / float(len(data['predict']))
        #count['truth'][0] + count['truth'][1] + count['truth'][2] + count['truth'][3] + count['truth'][4])

    return count

if __name__ == "__main__":
    #################################################
    #             FETCH DATA FROM FILES             #
    #################################################
    data = { 'length' : [], 'count' : [] }
    for i in range(START_LINK, MAX_LINK + 1):
        data['length'].append(fetchData(i))
    data['count'] = linkCounts(fetchData(-1))

    #################################################
    # CALCULATE/PLOT AVG DIFFERENCE IN LINK LENGTHS #
    #################################################
    avgLinkDifference = averageLinkDifference(data['length'])
    avgDiffLineData = []
    for i in range(len(avgLinkDifference)):
        avgDiffLineData.append(LineData(EPOCHS, avgLinkDifference[i], COLORS[i], str(START_LINK + i) + ' Links'))
    avgDiffPlot = AnalysisPlot(avgDiffLineData, 'epochs', 'Average Link Difference', 'Average Link Difference (Non-Occluded)', True, 'linkDifference_non_occluded.png', 'analysis/', False)
    avgDiffPlot.plot()

    #################################################
    #           PLOT LOSS IN LINK LENGTHS           #
    #################################################
    lossLineData = []
    for i in range(len(data['length'])):
        lossLineData.append(LineData(EPOCHS, data['length'][i]['loss'], COLORS[i], str(START_LINK + i) + ' Links'))
    lossPlot = AnalysisPlot(lossLineData, 'epochs', 'Loss', 'Loss (Non-Occluded)', True, 'loss_non_occluded.png', 'analysis/', False)
    lossPlot.plot()

    #################################################
    # CALCULATE/PLOT AVG DIFF IN TOTAL LINK LENGTHS #
    #################################################
    avgTotalLinkDifference = totalLinkDifference(data['length'])
    avgTotalDiffLineData = []
    for i in range(len(avgLinkDifference)):
        avgTotalDiffLineData.append(LineData(EPOCHS, avgTotalLinkDifference[i], COLORS[i], str(START_LINK + i) + ' Links'))
    avgTotalDiffPlot = AnalysisPlot(avgTotalDiffLineData, 'epochs', 'Average Total Link Difference', 'Average Difference in Links Total Length (Non-Occluded)', True, 'linkTotals_non_occluded.png', 'analysis/', False)
    avgTotalDiffPlot.plot()

    ######################################################
    # CALCULATE/PLOT AVG DIFF IN NORMALIZED LINK LENGTHS #
    ######################################################
    avgLinkDifferenceNormalized = averageLinkDifference(data['length'])
    avgDiffLineNormalizedData = []
    for i in range(len(avgLinkDifferenceNormalized)):
        normalizeValue = i + START_LINK
        for j in range(len(avgLinkDifferenceNormalized[i])):
            avgLinkDifferenceNormalized[i][j] = avgLinkDifferenceNormalized[i][j] / normalizeValue
        avgDiffLineNormalizedData.append(LineData(EPOCHS, avgLinkDifferenceNormalized[i], COLORS[i], str(START_LINK + i) + ' Links'))
    avgDiffNormalPlot = AnalysisPlot(avgDiffLineNormalizedData, 'epochs', 'Average Link Difference (Normalized)', 'Average Link Difference (Normalized) (Non-Occluded)', True, 'linkDifferenceNormalized_non_occluded.png', 'analysis/', False)
    avgDiffNormalPlot.plot()

    #################################################
    #              PLOT COUNT ACCURACY              #
    #################################################
    countAccLines = []
    for i in range(len(data['count']['accuracy'])):
        countAccLines.append(LineData(range(MAX_ACCURACY_EPOCH), data['count']['accuracy'][i], COLORS[i], str(START_LINK + i) + ' Links'))
    countAccLines.append(LineData(range(MAX_ACCURACY_EPOCH), data['count']['totalAccuracy'], 'k', 'Total Accuracy'))
    accCountPlot = AnalysisPlot(countAccLines, 'epochs', 'Accuracy', 'Accuracy in link count (Non-Occluded)', True, 'countAccuracy_non_occluded.png', 'analysis/', False)
    accCountPlot.plot()
