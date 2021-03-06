#!/bin/bash
############################################################
# Help                                                     #
############################################################
Help()
{
  # Display Help
  echo
  echo "Syntax: webotshpc_batch.sh [-d|r]"
  echo "options:"
  echo "h             Display script help"
  echo "j <string>    Job file directory"
  echo "n <int>       Number of runs in the sequence"
  echo "p <bool>     Parallelization flag"
  

  echo
}

############################################################
############################################################
# Main program                                             #
############################################################
############################################################

# Set default variables
Directory='./'
Num_runs=1

############################################################
# Process the input options. Add options as needed.        #
############################################################
# Get the options
while getopts "hj:n:" o; do
  case "${o}" in
    h) # display help
    Help
    exit;;
    j) # Enter a directory
    Directory=${OPTARG};;
    n) # Enter the number of runs
    Num_runs=${OPTARG};;
    \?) # Invalid option
    echo "Error: Invalid option"
    Help
    exit;;
  esac
done
shift $((OPTIND-1))

# Intro Message and User Input Validation
echo "Webots.HPC: Batch of Simulations Script";
echo
echo "---------- User Input Validation ----------";
echo "Job (.pbs) File Directory: $Directory"
echo "Number of Runs: $Num_runs"
echo "-------------------------------------------";
echo


# Triggering Jobs
job=$(qsub $Directory)
echo "Triggering Job 1: $job"
for ((i = 2; i <= Num_runs; i++))
do
	job_next=$(qsub -W depend=afterany:$job $Directory)
	echo "Triggering Job $i: $job_next"
	job=$job_next
done
