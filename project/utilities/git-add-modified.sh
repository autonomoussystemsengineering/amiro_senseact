#!/bin/bash

set -e

DIR="."
LG='\033[1;32m'
DG='\033[0;32m'
NC='\033[0m'

#          Get the filelist     Get modified    Remove prefix
for F in $(git status --short ${DIR} | grep -e "^.M" | sed 's#^.M\s*##g'); do
  while [[ true ]]; do
  echo -ne "Add ${LG}${F}${NC} [${DG}Y${NC}es/${DG}n${NC}o/${DG}d${NC}iff/diff${DG}t${NC}ool]:"
  read A
  if [[ ${A} == "" || ${A} == "y" || ${A} == "Y" ]]; then
    git add ${F}
    break
  elif [[ ${A} == "n" ]]; then
   break
  elif [[ ${A} == "d" ]]; then
   git diff ${F}
  elif [[ ${A} == "t" ]]; then
   git difftool ${F}
  fi
  done
done

