ps aux | grep hyp | grep -v grep | ruby -e "STDIN.readlines.each{|x| puts x.split(' ')[1].strip}" | xargs kill -9
