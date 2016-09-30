NINETY=1.571
ONE_EIGHTY=3.14159

JOINT1=`rostopic echo /jaco_arm_driver/out/joint_state | ruby -e "x=STDIN.readline; while x; if x.include?('position'); puts /\[([\-\d\.]+)/.match(x)[1]; break; end; x = STDIN.readline; end"`

echo Straightening arm
rosrun jaco_demo joint_angle_workout.py jaco $JOINT1 -$NINETY -$NINETY $NINETY $ONE_EIGHTY -1.8

