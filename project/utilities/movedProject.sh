#!/bin/bash

# load actual and start directory
oldDirectory=$(pwd)
startDirectory=$MUROX_PROJECT

allFolders=(act demo includes process sandbox sense tools)

# check start directory
sdirArr=$(echo $startDirectory | tr "/" "\n")
sdirLength=0
startDirectory="/"
for x in $sdirArr; do
  if [[ ${#x} -gt 0 ]]; then
    sdirLength=$(($sdirLength+1))
    startDirectory=$(echo "${startDirectory}${x}/")
  fi
done

# function for usage information
manual () {
  echo "Call: ./moveProject.sh <old path> <new path> OPTION"
  echo ""
  echo "Necessary arguments:"
  echo " <old path>   Path to old project (starting at ${startDirectory})."
  echo " <new path>   Path to new project (starting at ${startDirectory})."
  echo ""
  echo "Options:"
  echo " -h [--help]  Displays this manual."
  echo " -r           The replacements will be done without any notice about the changes."
  echo " -p           The replacement changes will be printed without doing the replacement."
}

# check if both parameters are given
if [ -z "${1}" ]; then
  echo "There are missing arguments! Please have a look into the following manual:"
  echo ""
  manual
  exit 1
fi

if [ -z "${2}" ]; then
  echo "There are missing arguments! Please have a look into the following manual:"
  echo ""
  manual
  exit 1
fi

# check third parameter
if [ -z "${3}" ]; then
  echo "Missing option! Please have a look into the following manual:"
  echo ""
  manual
  exit 1
elif [[ ( "${3}" == "-h" ) || ( "${3}" == "--help" ) ]]; then
  manual
  exit 1
elif [ "${3}" == "-r" ]; then
  justPrint=false
elif [ "${3}" == "-p" ]; then
  justPrint=true
else
  echo "Unknown option! Please have a look into the following manual:"
  echo ""
  manual
  exit 1
fi

# prepare old project's path and reference string for sed
oldArr=$(echo $1 | tr "/" "\n")
oldPath="}"
oldRef="\[\["
oldProject=""
for x in $oldArr; do
  if [[ ${#x} -gt 0 ]]; then
    oldPath=$(echo "$oldPath\/$x")
    oldRef=$(echo "$oldRef$x:")
    oldProject=$(echo "$oldProject$x/")
  fi
done
oldPath=$(echo "$oldPath ")

# prepare new project's path and reference string for sed
newArr=$(echo $2 | tr "/" "\n")
newPath="}"
newRef="\[\["
newProject=""
for x in $newArr; do
  if [[ ${#x} -gt 0 ]]; then
    newPath=$(echo "$newPath\/$x")
    newRef=$(echo "$newRef$x:")
    newProject=$(echo "$newProject$x/")
  fi
done
newPath=$(echo "$newPath ")

# check if new project exists
fullProjectPath=$(echo "${startDirectory}${newProject}")
if [[ -d "$fullProjectPath" ]]; then
  echo "Replace old project $oldProject by new project $newProject."
  if $justPrint; then
    echo "The following changes would be done:"
    echo ""
  fi
else
  echo "The new project $fullProjectPath doesn't exist!"
  exit 1
fi

# define function for path correction
getCurrentPath () {
  actPath=$(pwd)
  pathArr=$(echo $actPath | tr "/" "\n")
  partCounter=0
  actPath=""
  for x in $pathArr; do
    if [[ ${#x} -gt 0 ]]; then
      if [[ $partCounter -ge $sdirLength ]]; then
        actPath=$(echo "${actPath}${x}/")
      fi
      partCounter=$(($partCounter+1))
    fi
  done
}

# define function for checking directories recursively
checkDirectory () {
  for d in *; do
    # create file path
    getCurrentPath
    actFile=$(echo "${actPath}${d}")

    # check for subdirectory
    if [ -d $d ]; then
      cd "$d"
      checkDirectory
      cd ..

    # check for CMakeLists
    # just print change notice
    elif [[ $d == CMakeLists.txt && "$justPrint" = true ]]; then
      lineB=$(sed -n "/$oldPath/p" $d)
      lineA=$(sed -n "s/$oldPath/$newPath/p" $d)
      if [[ ${#lineB} -gt 0 ]]; then
        echo "${actFile}:"
        echo "${lineB}"
        echo "-> to:"
        echo "${lineA}"
        echo ""
      fi
    # do change
    elif [[ $d == CMakeLists.txt ]]; then
      sed -i "s/$oldPath/$newPath/g" $d

    # check for txt file (documentation)
    # just print change notice
    elif [[ $d == *.txt && "$justPrint" = true ]]; then
      lineB=$(sed -n "/$oldRef/p" $d)
      lineA=$(sed -n "s/$oldRef/$newRef/p" $d)
      if [[ ${#lineB} -gt 0 ]]; then
        echo "${actFile}:"
        echo "${lineB}"
        echo "-> to:"
        echo "${lineA}"
        echo ""
      fi
    # do change
    elif [[ $d == *.txt ]]; then
      sed -i "s/$oldRef/$newRef/g" $d
    fi
  done
  rm -f *~
}

# change to main project directory
cd $startDirectory

# check all defined subdirectories
for folder in ${allFolders[*]}; do
  cd $folder
  checkDirectory
  cd $startDirectory
done

# change back
cd $oldDirectory

if [[ "$justPrint" = false ]]; then
  echo ""
  echo "Replacement done."
fi

