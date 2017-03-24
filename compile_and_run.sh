reset

PROG="attachment"
CORES="4"
COMP="mpicc -o $PROG attachment.c"
RUN="mpirun --host localhost -np $CORES ./$PROG datasets/diverse_movement_skeleton.csv datasets/diverse_movement_pcl.csv"

clear; $COMP && $RUN


