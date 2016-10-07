ps aux | grep "roslaunch kinect2_bridge" | grep -v grep | ruby -e "STDIN.readlines.each{|x| puts x.split.join(' ')}" | cut -d' ' -f2 | xargs kill -9

./kill.sh

ps aux | grep "jaco_arm_driver" | grep -v grep | ruby -e "STDIN.readlines.each{|x| puts x.split.join(' ')}" | cut -d' ' -f2 | xargs kill -9

#ps aux | grep -e "roscore|rosmaster" | grep -v grep | ruby -e "STDIN.readlines.each{|x| puts x.split.join(' ')}" | cut -d' ' -f2 | xargs kill -9

