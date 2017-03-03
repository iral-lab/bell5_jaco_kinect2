ps aux | grep ${1}.py | grep python| grep -v grep | ruby -e "STDIN.readlines.each{|x| puts x.split[1]}" | xargs kill -9
