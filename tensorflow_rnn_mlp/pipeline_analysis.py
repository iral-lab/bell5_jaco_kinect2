import sys, cPickle, code, math

def correct_phase_1_by_true(data):
	correct_num_links = {}
	total_per_num_links = {}
	predicted = {}
	
	for row in data:
		true_num_links = row[0].index(1) + 2 # offset
		if not true_num_links in correct_num_links:
			correct_num_links[true_num_links] = 0
			total_per_num_links[true_num_links] = 0
			predicted[true_num_links] = {2:0, 3:0, 4:0, 5:0, 6:0}
		
		total_per_num_links[true_num_links] += 1
		if row[4]:
			correct_num_links[true_num_links] += 1
		
		predicted[true_num_links][row[3]] += 1
		
		#code.interact(local=dict(globals(), **locals()))
		
	
	print "Percentage correct by phase 1 one-hot:"
	for key in sorted(correct_num_links.keys()):
		percentage = 100.0 * correct_num_links[key] / total_per_num_links[key]
		print str(key)+"-link: one-hot accuracy:",correct_num_links[key],"/",total_per_num_links[key],"=",round(percentage,3),'%, predicted as:',predicted[key]
		
	#code.interact(local=dict(globals(), **locals()))
	pass

def vector_length(v):
	return math.sqrt(sum([v[i]*v[i] for i in xrange(len(v))]))

def compute_robot_lengths_in_phase_2(all_data):
	keys = [2,3,4,5,6]
	
	overall_scatter_file_name = "./pipeline_pickles/length_scatter_all.tsv"
	
	with open(overall_scatter_file_name,'w') as overall_scatter_handle:
		overall_scatter_handle.write("n-Links\tTrue Length\tPredicted Length\n")
		for key in keys:
			if not key in all_data:
				continue
		
			data = all_data[key]
			
			total_prediction_delta = 0.0
			
			correct_prediction_delta = 0.0
			correct_predictions = 0
			
			incorrect_prediction_delta = 0.0
			incorrect_predictions = 0
		
			scatter_file_name = "./pipeline_pickles/length_scatter_"+str(key)+".tsv"
			with open(scatter_file_name,"w") as output_scatter_handle:
				output_scatter_handle.write("True Length\tPredicted Length\n")
				for row in data:
					true_robot_lengths = row[1]
					true_robot_length = vector_length(true_robot_lengths)
			
					predicted_robot_lengths = row[6]
					predicted_robot_length = vector_length(predicted_robot_lengths)
					
					prediction_delta = abs(true_robot_length - predicted_robot_length)
					total_prediction_delta += prediction_delta
					
					if row[4]:
						correct_predictions += 1
						correct_prediction_delta += prediction_delta
					else:
						incorrect_predictions += 1
						incorrect_prediction_delta += prediction_delta
						
				
					output_scatter_handle.write("\t".join([str(true_robot_length), str(predicted_robot_length)])+"\n")
					overall_scatter_handle.write("\t".join([str(key), str(true_robot_length), str(predicted_robot_length)])+"\n")
			
			# print key,"scatter written to",scatter_file_name
			
			# when correct

			print "\tCorrectly predicted",str(key)+"-link robots:",correct_predictions
			print "\tTotal prediction delta:", correct_prediction_delta
			print "\taverage delta:",round(correct_prediction_delta / correct_predictions,2)
			print
			
			# average
			
			print "\tAll predicted",str(key)+"-link robots:",len(data)
			print "\tTotal prediction delta:", total_prediction_delta
			print "\taverage delta:",round(total_prediction_delta / len(data),2)
			print
			# when incorrect
			
			print "\tIncorrectly predicted",str(key)+"-link robots:",incorrect_predictions
			print "\tTotal prediction delta:", incorrect_prediction_delta
			print "\taverage delta:",round(incorrect_prediction_delta / incorrect_predictions,2)
			print "------------------"
			print


if '__main__' == __name__:
	ALL = 'all'

	files = {
		ALL : "pipeline1_results_1536982019.pickle",
 		2 : "pipeline2_results_1536982190_2.pickle",
 		3 : "pipeline2_results_1536982256_3.pickle",
 		4 : "pipeline2_results_1536982291_4.pickle",
 		5 : "pipeline2_results_1536982311_5.pickle",
		6 : "pipeline2_results_1536982336_6.pickle",
	}
	folder = "./pipeline_pickles/"
	
	loaded = {}
	for k,v in files.items():
		print "loading",k,v
		loaded[k] = cPickle.load(open(folder + v))
	# code.interact(local=dict(globals(), **locals()))
	print "\n\n"
	
	print "Starting correct_phase_1_by_true"
	correct_phase_1_by_true(loaded[ALL])
	
	print "\n\n"
	
	print "Starting compute_robot_lengths_in_phase_2"
	compute_robot_lengths_in_phase_2(loaded)
	
	
	
	
	

	

