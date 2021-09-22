#!/bin/bash

# Assigning variables
Job_name='webots.hpc_job'
Num_instances=1
Queue=''
Select=1
Ncpus=1
Memory="4gb"
Interconnect="hdr"
Walltime="00:45:00"



while getopts "hj:n:q:s:c:m:i:w" o; do
  case "${o}" in
    h) # display help
    Help
    exit;;
    j) # Enter a job name
      Job_name=${OPTARG};;
    n) # Enter the number of instances
      Num_instances=${OPTARG};;
    q) # Enter job queue
      Queue=${OPTARG};;
    s) # Enter select (# of nodes)
      Select=${OPTARG};;
    c) # Enter number of CPUs
      Ncpus=${OPTARG};;
    m) # Enter amount of memory
      Memory=${OPTARG};;
    i) # Enter interconnect
      Interconnect=${OPTARG};;
    w) # Enter walltime
      Walltime=${OPTARG};;
    \?) # Invalid option
    echo "Error: Invalid option"
    Help
    exit;;
  esac
done
shift $((OPTIND-1))

#PBS -N $Job_name
#PBS -l select=$Select:ncpus=$Ncpus:mem=$Memory:interconnect=$Interconnect,walltime=$Walltime
#PBS -J 1-$Num_instances
#PBS -q $Queue


#echo Generating new random routes...
#singularity exec -B $TMPDIR:$TMPDIR ../webots_sumo.sif /usr/local/webots/projects/default/resources/sumo/bin/duarouter --route-files /home/mwfranc/Webots.HPC/Simulations/HighwayMerge/worlds/LinkonMKZmyMap_$(($PBS_ARRAY_INDEX % 8))_net/sumo.flow.xml --net-file /home/mwfranc/Webots.HPC/Simulations/HighwayMerge/worlds/LinkonMKZmyMap_$(($PBS_ARRAY_INDEX % 8))_net/sumo.net.xml --output-file /home/mwfranc/Webots.HPC/Simulations/HighwayMerge/worlds/LinkonMKZmyMap_$(($PBS_ARRAY_INDEX % 8))_net/#sumo.rou.xml --randomize-flows true --seed $RANDOM

#echo Starting Webots on 'hostname'
#singularity exec -B $TMPDIR:$TMPDIR ../webots_sumo.sif xvfb-run -a webots --stdout --stderr --batch --mode=realtime Webots.HPC/Simulations/HighwayMerge/worlds/LinkonMKZmyMap_$(($PBS_ARRAY_INDEX % 8)).wbt
