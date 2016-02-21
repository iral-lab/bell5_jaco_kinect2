NINETY=1.571
ONE_EIGHTY=3.14159

./straighten_arm.sh

echo Lowering
rosrun jaco_demo joint_angle_workout.py jaco -0.5 0 -$ONE_EIGHTY $NINETY $ONE_EIGHTY -1.8
