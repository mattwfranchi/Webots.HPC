#!/bin/bash
job=$(qsub sim_parallel_6x2.pbs)
echo $job
for ((i = 2; i <= 10; i++))
do
	job_next=$(qsub -W depend=afterany:$job sim_parallel_6x2.pbs)
	echo $job_next
	job=$job_next
done

echo "done"


