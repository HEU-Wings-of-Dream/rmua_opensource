#!/bin/bash
cd /home/ubuntu/Documents/RoboMaster/HEU_Vision_2020_MDVS/
gnome-terminal "HEU_Vision_2020_MDVS" -x bash -c "./startup.sh; exec bash"

#sleep 5
#procID=`pgrep HEU_Vision_2020_MDVS`
#kill "$procID"

#sleep 5
#procID=`pgrep HEU_Vision_2020_MDVS`
#kill "$procID"

#while [ 1 ]
#do
#	sleep 100
#	procID=`pgrep HEU_Vision_2020_MDVS`
#	kill "$procID"
#done &
