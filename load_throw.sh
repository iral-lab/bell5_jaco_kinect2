NINETY=1.571
ONE_EIGHTY=3.14159

#./straighten_arm.sh

echo Lowering
rosrun jaco_demo joint_angle_workout.py jaco -0.5 0 -$NINETY 0.5 1.3 -1.8
rosrun jaco_demo grip_workout.py jaco 3000 3000 3000



