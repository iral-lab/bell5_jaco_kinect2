while true; do
	#rsync -v -e "ssh -i ~/.ssh/neilrbell_solo_20160113.pem" ec2-user@${1}:~/bell5_jaco_kinect2/caches/* caches/
	#rsync -v -e "ssh -i ~/.ssh/neilrbell_solo_20160113.pem" ec2-user@${1}:~/bell5_jaco_kinect2/best_* .
	rsync -v -e "ssh -i ~/.ssh/neilrbell_solo_20160113.pem" ec2-user@${1}:~/bell5_jaco_kinect2/attachment_scored_outputs.csv .
	echo "Done, sleeping"
	# sleep for 10 minutes
	sleep 600
done
