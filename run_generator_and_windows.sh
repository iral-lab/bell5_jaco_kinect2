gnome-terminal -e "bash -c 'htop'"
sleep 1

gnome-terminal -e "bash -c 'while true; do sensors; sleep 5; done'"
sleep 1

gnome-terminal -e "bash -c 'while true; do echo; echo;echo; date; ls -1 clouds/ | wc -l; du -hc clouds/; ls -1 clouds/ | cut -d'_' -f1 | sort | uniq -c; sleep 60; done'"
sleep 1

gnome-terminal -e "bash -c 'while true; do time python generator.py; sleep 5; done'"
sleep 1


