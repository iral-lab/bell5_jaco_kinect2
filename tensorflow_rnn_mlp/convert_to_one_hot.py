import sys, cPickle, code
sys.path.append('../simulator/')
from generator import MAX_LINKS

input = "_classifier_input_RNN.pickle"
output = "_classifier_input_RNN_ONE_HOT.pickle"

input_data = cPickle.load(open(input,'r'))

observations,labels = input_data

for i,label in enumerate(labels):
	new_label = [0] * MAX_LINKS
	num_links = label[0]
	new_label[num_links] = 1
	labels[i] = new_label

cPickle.dump((observations,labels), open(output,'w'))
