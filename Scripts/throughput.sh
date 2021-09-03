#!/bin/bash

start=$SECONDS
duration=$(( SECONDS - start ))

while [ $duration -le 14400 ]
do
	if [ $(( duration % 1800 )) -eq 0 ]
	then
		fc=$( ls ~/Webots.HPC/output/*.csv | wc -l )
		echo $duration, $fc
	fi
	duration=$(( SECONDS - start ))
done	


