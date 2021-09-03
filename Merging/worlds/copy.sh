#!/bin/bash
# Copy out Webots world 8 times to uniquely-named folders / files

for i in {0..7}
do
	if [ $i -ne 1 ]
	then
		echo "World Number: $i"
		cp LinkonMKZmyMap_1.wbt LinkonMKZmyMap_${i}.wbt
		cp -r ./LinkonMKZmyMap_1_net/ ./LinkonMKZmyMap_${i}_net
	fi
done
