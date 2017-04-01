reset

if [ -z $1 ];
then
	echo "must give number of cores to use"
	exit
fi


PROG="attachment"
CORES=$1
COMP="mpicc -o $PROG attachment.c"
RUN="mpirun --host localhost -np $CORES ./$PROG run short_skeleton.csv short_pcl.csv output.csv"

clear; $COMP && $RUN


