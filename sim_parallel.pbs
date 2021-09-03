#!/bin/bash
#PBS -N webots
#PBS -l select=1:ncpus=5:mem=93gb:interconnect=hdr,walltime=00:10:00
#PBS -J 1-48
#PBS -q dicelab
module purge
echo Generating new random routes...
for i in {0..7}
do
  singularity exec -B $TMPDIR:TMPDIR webots_sumo.sif /usr/local/webots/projects/default/resources/sumo/bin/duarouter --route-files /home/mwfranc/Webots.HPC/Simulations/HighwayMerge/worlds/LinkonMKZmyMap_0_net/sumo.flow.xml --net-file /home/mwfranc/Webots.HPC/Simulations/HighwayMerge/worlds/LinkonMKZmyMap_0_net/sumo.net.xml --output-file /home/mwfranc/Webots.HPC/Simulations/HighwayMerge/worlds/LinkonMKZmyMap_0_net/sumo.rou.xml --randomize-flows true --seed 40
done
echo Starting Webots on 'hostname'
singularity exec -B $TMPDIR:$TMPDIR webots_sumo.sif xvfb-run -a webots --stdout --stderr --batch --mode=realtime Webots.HPC/Simulations/HighwayMerge/worlds/LinkonMKZmyMap_$(($PBS_ARRAY_INDEX % 8)).wbt
