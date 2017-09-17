ps aux | grep python | grep classifier | grep -v grep | ruby -e "STDIN.readlines.each{|x| puts x.split.join(' ')}" | cut -d' ' -f2 | xargs  kill -9
