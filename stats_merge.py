import sys, os

IN_DIR = "run_stats/"

if "__main__" == __name__:
	
	train_cost_index = 2
	
	labels = None
	data = []
	first_time = True
	
	for folder in sorted(os.listdir(IN_DIR)):
		if not os.path.isdir(IN_DIR + folder):
			continue
		
		folder_parts = folder.split("_")
		n_layers, n_nodes = [int(x) for x in folder_parts[2:]]
		
		run_label = str(n_layers) + "  " + str(n_nodes)
		while len(data) < 1:
			data.append([])
		if first_time:
			data[0].append("") # empty upper-left cell
			first_time = False
		
		if True:
			valid_suffixes = [
				"_20", "_80", "_140", "_190",
			]
			valid = False
			for suffix in valid_suffixes:
				if folder[-1 * len(suffix):] == suffix:
					valid = True
					break
			if not valid:
				continue
		data[0].append(run_label)
				
		folder = IN_DIR + folder + "/"

		for file in sorted(os.listdir(folder)):
			file = folder + file
			print file
			lines = open(file, 'r').readlines()
			
			lines = lines[1:] # strip header
			
			parts = []
			for line in lines:
				items = line.strip().split("\t")
				items = [float(x) for x in items]
				parts.append(items)
			
			if not labels:
				labels = []
				for row in parts:
					epoch, batch = [int(x) for x in row[:2]]
					label = epoch * 100 + batch
					# print epoch, batch, label
					labels.append(label)
				while len(data) < len(labels)+1:
					data.append([])
				for i, label in enumerate(labels):
					data[i+1].append(label)
			
			for i,row in enumerate(parts):
				data[i+1].append(row[train_cost_index])
			
			
			# print data
			
			# break
		# break
	
	to_write = []
	for i,row in enumerate(data):
		# print only after final training
		if not row[0] == "" and not row[0] % 100 == 9:
			continue
		
		row = [str(x) for x in row]
		to_write.append("\t".join(row))
	open("merged_run_stats.tsv", "w").write("\n".join(to_write))
	
	
	
	