reset

if [ -z $1 ];
then
	echo "must give number of cores to use"
	exit
fi


PROG="attachment"
CORES=$1
SKELETON="short_skeleton.csv"
PCL="short_pcl.csv"
OUTPUT="output.csv"
COMP="mpicc -o $PROG attachment.c"
RUN="mpirun --host localhost -np $CORES ./$PROG run $SKELETON $PCL $OUTPUT"
BEST="mpirun --host localhost -np $CORES ./$PROG best $SKELETON $PCL $OUTPUT"

clear; $COMP && $RUN && $BEST

