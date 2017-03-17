PROG="attachment"
COMP="mpicc -o $PROG attachment.c"
RUN="mpirun --host localhost -np 4 ./$PROG"

$COMP && $RUN


