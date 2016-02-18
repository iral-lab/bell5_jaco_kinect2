import sys

input = """
270.606628418
150.09375
22.389705658
268.43182373
5.79545450211
100.227272034


"""

for line in input.split("\n"):
    line = line.strip()
    if line == "":
        continue
    degrees = float(line)
    print degrees * 0.0174533

