#!/bin/bash
############################################################
# Help                                                     #
############################################################
Help()
{
   # Display Help
   echo "Add description of the script functions here."
   echo
   echo "Syntax: scriptTemplate [-g|h|v|V]"
   echo "options:"
   echo "g     Print the GPL license notification."
   echo "h     Print this Help."
   echo "v     Verbose mode."
   echo "V     Print software version and exit."
   echo
}

############################################################
############################################################
# Main program                                             #
############################################################
############################################################

# Set default variables
Directory="./"
Num_runs=1

while getopts ":dr:" option; do
  case $option in
    d) Directory=$OPTARG;;
    r) Num_runs=$OPTARG;;
  esac
done


echo "Webots.HPC: Batch of Simulations Script";
echo "User Input Validation: ";
echo "Simulation Directory: $directory";
echo "Number of Runs: $num_runs";
