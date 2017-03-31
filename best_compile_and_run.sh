reset

if [ -z $1 ];
then
	echo "must give number of cores to use"
	exit
fi


PROG="attachment"
CORES=$1
COMP="mpicc -o $PROG attachment.c"
RUN="mpirun --host localhost -np $CORES ./$PROG best datasets/diverse_movement_skeleton.csv datasets/diverse_movement_pcl.csv output_best.csv"

clear; $COMP && $RUN


