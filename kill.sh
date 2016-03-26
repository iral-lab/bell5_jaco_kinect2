ps aux | grep bell5_jaco | grep -v grep | grep -v gedit | ruby -e "STDIN.readlines.each{|x| puts x.split.join(' ')}" | cut -f 2 -d ' ' | xargs kill -9
