zcat diverse_movement.bag.a* > diverse_movement.bag
while true; do rosbag play -r 0.75 diverse_movement.bag; sleep 1; done

