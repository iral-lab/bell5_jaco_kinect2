NINETY=1.571
ONE_EIGHTY=3.14159

echo Throwing

rosrun jaco_demo joint_angle_workout.py jaco 0.5 0 0 $NINETY $ONE_EIGHTY -1.8

#echo Lowering
#rosrun jaco_demo joint_angle_workout.py jaco 0.5 -$ONE_EIGHTY 0 $NINETY $ONE_EIGHTY -1.8
