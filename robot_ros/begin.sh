gnome-terminal -e "roscore"
sleep 2
gnome-terminal -e "bash -c 'while true; do rosrun jaco_driver jaco_arm_driver; sleep 1; done'"
sleep 2
gnome-terminal -e "bash -c 'while true; do roslaunch kinect2_bridge kinect2_bridge.launch depth_method:=cpu; sleep 1; done'"


check_cmd="rostopic info /jaco_arm_driver/out/joint_state"

code=1
while [ $code -ne 0 ]
do
  sleep 1
  echo Verifying JACO is broadcasting and ready
  $check_cmd > /dev/null
  code=$?
done

echo Found arm

./init_arm.sh

echo Ready
