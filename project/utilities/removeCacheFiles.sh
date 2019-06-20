#!/bin/bash

# Get current and project directory
oldDirectory=$(pwd)
startDirectory=$MUROX_PROJECT

# function for usage information
manual () {
  echo "Call: ./removeCacheFiles.sh [OPTION]"
  echo ""
  echo "On default the command 'rm -i' is called in the project directory."
  echo "Current project directory: $startDirectory"
  echo "The file description is recursive:"
  echo " - *~"
  echo " - */*~"
  echo " - ..."
  echo " - *({/*}^10)~"
  echo ""
  echo "Options:"
  echo " -h [--help]  Displays this manual."
  echo " -f           Calls 'rm -f' instead of 'rm -i'."
}

# check if first parameter is given
if [ -z "${1}" ]; then
  forcerm=false
else
  if [[ ( "${1}" == "-h" ) || ( "${1}" == "--help" ) ]]; then
    manual
    exit 1
  elif [ "${1}" == "-f" ]; then
    forcerm=true
  else
    echo "Unknown option! Please have a look into the following manual:"
    echo ""
    manual
    exit 1
  fi
fi

# Define command
if $forcerm; then
  rmpath="rm -f "
else
  rmpath="rm -i "
fi
nextDepth="*/"
fileend1="*~"
fileend2=".*~"

# Move to project directory
cd $startDirectory

# Remove all cache files
for depth in {1..10}; do
  curcom=$(echo "${rmpath}${fileend1}")
  $($curcom)
  curcom=$(echo "${rmpath}${fileend2}")
  $($curcom)
  rmpath=$(echo "${rmpath}${nextDepth}")
done

# Move to current directory
cd $oldDirectory
