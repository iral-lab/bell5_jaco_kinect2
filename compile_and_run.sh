PROG="attachment"
COMP="mpicc -o $PROG attachment.c"
RUN="mpirun --host localhost -np 4 ./$PROG datasets/diverse_movement_skeleton.csv datasets/diverse_movement_pcl.csv"

clear; $COMP && $RUN


