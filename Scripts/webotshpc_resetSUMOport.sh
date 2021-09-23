awk '$0 ~ /port/ { $1 = "  " $1; $2 = 8873 } 1' ~/Webots.HPC/Simulations/HighwayMerge/worlds/LinkonMKZmyMap.wbt > tmp$$ && mv tmp$$ ~/Webots.HPC/Simulations/HighwayMerge/worlds/LinkonMKZmyMap.wbt
