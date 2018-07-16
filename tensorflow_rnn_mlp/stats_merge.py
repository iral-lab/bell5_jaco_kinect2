import sys, os

IN_DIR = "run_stats/"

RUN_MLP, RUN_RNN = range(2)
RUN_TYPE = RUN_RNN

MLP_SUFFIXES = None #["_20", "_80", "_140", "_190",]
RNN_SUFFIXES = ["_20", "_60", "_100", "_140", "_180"]

if "__main__" == __name__:
	
	train_cost_index = 2
	
	labels = None
	data = []
	first_time = True
	
	for folder in sorted(os.listdir(IN_DIR)):
		if not os.path.isdir(IN_DIR + folder):
			continue
		
		if RUN_TYPE == RUN_RNN and not "RNN" in folder:
			continue
		if RUN_TYPE == RUN_MLP and not "MLP" in folder:
			continue
		
		folder_parts = folder.split("_")
		n_layers, n_nodes = [int(x) for x in folder_parts[3:]]
		
		run_label = str(n_layers) + " layers,  " + str(n_nodes) + " nodes/layer"
		while len(data) < 1:
			data.append([])
		if first_time:
			data[0].append("") # empty upper-left cell
			first_time = False
		
		if True:
			valid_suffixes = MLP_SUFFIXES if RUN_TYPE == RUN_MLP else RNN_SUFFIXES
			valid = False
			if not valid_suffixes:
				valid = True
			else:
				for suffix in valid_suffixes:
					if folder[-1 * len(suffix):] == suffix:
						valid = True
						break
			if not valid:
				continue
				
		folder = IN_DIR + folder + "/"

		
		for file in sorted(os.listdir(folder)):
			has_lines = False
			file = folder + file
			print file
			lines = open(file, 'r').readlines()
			
			lines = lines[1:] # strip header
			
			parts = []
			for line in lines:
				while '  ' in line:
					line = line.replace('  ',' ')
				line = line.replace(' ',"\t")
				items = line.strip().split("\t")
				items = [float(x) for x in items]
				parts.append(items)
			# print len(parts)
			# print parts[0]
			
			if not labels:
				labels = []
				for row in parts:
					epoch = row[0]
					labels.append(epoch)
				while len(data) < len(labels)+1:
					data.append([])
				for i, label in enumerate(labels):
					data[i+1].append(label)
			
			for i,row in enumerate(parts):
				has_lines = True
				data[i+1].append(row[train_cost_index])
			if has_lines:
				data[0].append(run_label)
			else:
				print "\tNo data!"
	to_write = []
	for i,row in enumerate(data):
		# print only after final training
		row = [str(x) for x in row]
		to_write.append("\t".join(row))
	type = "_RNN" if RUN_TYPE == RUN_RNN else "_MLP"
	open("merged_run_stats"+type+".tsv", "w").write("\n".join(to_write))
	
	
	
	