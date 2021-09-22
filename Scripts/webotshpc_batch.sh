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
  echo "d <string>    Simulation file directory"
  echo "r <int>       Number of runs in the sequence"
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
while getopts "hd:r:" o; do
  case "${o}" in
    h) # display help
    Help
    exit;;
    d) # Enter a directory
    Directory=${OPTARG};;
    r) # Enter the number of runs
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
echo "Simulation Directory: $Directory"
echo "Number of Runs: $Num_runs"
echo "-------------------------------------------";
echo
