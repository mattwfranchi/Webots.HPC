#!/bin/bash
############################################################
# Help                                                     #
############################################################
Help()
{
  # Display Help
  echo
  echo "Syntax: webotshpc_parallelize.sh [-j|n|q|s|c|m|i|w]"
  echo "options:"
  echo "h             Display script help"
  echo "j <string>    Job name"
  echo "n <int>       Number of instances to trigger in batch"
  echo "q <string>    Queue name"
  echo "s <int>       Select (# of compute nodes to distribute across)"
  echo "c <int>       Number of CPUs to request per node"
  echo "m <string>    Amount of memory to request per node"
  echo "i <string>    Type of interconnect to request"
  echo "w <string>    Walltime to request"
  echo
}

############################################################
############################################################

############################################################
############################################################
# Main program                                             #
############################################################
############################################################


# Assigning variables
Job_name='webots.hpc_job'
Num_instances=4
Queue=''
Select=1
Ncpus=5
Memory="93gb"
Interconnect="hdr"
Walltime="00:45:00"

# Processing Command Line Arguments
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


# Intro Message
echo "Webots.HPC: Parallelized Job Submission Script"
echo
echo "-------------- Summary of Resources Requested --------------"
echo "Number of Instances to Trigger: 		$Num_instances"
echo "Number of Nodes to Request (Select): 	$Select"
echo "Number of CPUs to Request:		$Ncpus"
echo "Amount of Memory to Request:		$Memory"
echo "Type of Interconnect:			$Interconnect"
echo "Walltime Requested:			$Walltime"
echo "Job Queue (If Specified):			$Queue"
echo "------------------------------------------------------------"
echo

# Triggering Customized Job

if [ -z "$Queue" ]
then
	echo
	echo "No queue specified, so using default." 	
	qsub -J 1-$Num_instances -l select=$Select:ncpus=$Ncpus:mem=$Memory:interconnect=$Interconnect,walltime=$Walltime $Job_name
else
	qsub -q $Queue -J 1-$Num_instances -l select=$Select:ncpus=$Ncpus:mem=$Memory:interconnect=$Interconnect,walltime=$Walltime $Job_name
fi

