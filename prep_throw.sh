NINETY=1.571
ONE_EIGHTY=3.14159

./straighten_arm.sh

echo Rotating

rosrun jaco_demo joint_angle_workout.py jaco 0.5 -$NINETY -$NINETY $NINETY $ONE_EIGHTY -1.8

echo Lowering
rosrun jaco_demo joint_angle_workout.py jaco 0.5 -$ONE_EIGHTY -$ONE_EIGHTY $NINETY $ONE_EIGHTY -1.8
