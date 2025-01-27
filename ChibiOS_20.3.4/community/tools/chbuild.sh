#!/bin/bash

#
# Author: Fabien Poussin
# Date: 21/04/2017
# Version: 1.0
#

renice +10 $$
JOBS=$(nproc)
SKIP_ARRAY=(Win32)
RETCODE=0

function chbuild {
  projects=$(find $1 -name Makefile -printf '%h ')
  OK=0
  NOK=0
  FAIL=()
  SUCCESS=()
  SKIPPED=()
  for t in $projects
  do
    if [[ -f "${t}/.skip" ]]; then
      printf "SKIPPING: ${t}\n"
      SKIPPED+=($t)
      continue
    fi
    pushd $t > /dev/null
    if [[ ! -z ${CH_CLEAN+x} ]]; then
        printf "CLEANING: ${t}\n"
        make clean
    fi
    printf "BUILDING: ${t}\n"
    if [[ -z "${CH_PATH+x}" || -z "${CHC_PATH+x}" ]]; then
        make --quiet -j $JOBS > /dev/null
    else
        make CHIBIOS=$CH_PATH CHIBIOS_CONTRIB=$CHC_PATH --quiet -j $JOBS > /dev/null
    fi
    if [ $? -ne 0 ]; then
      ((NOK++))
      FAIL+=($t)
      RETCODE=1
    else
      ((OK++))
      SUCCESS+=($t)
    fi
    popd > /dev/null
  done
  printf "\n${1}: ${OK} builds ok, ${NOK} builds failed\n"
  printf 'FAIL: %s\n' "${FAIL[@]}"
  printf 'SKIPPED: %s\n' "${SKIPPED[@]}"
  printf "\n"
  return $NOK
}

if [ -z "$1" ]
  then
    printf "This script looks for Makefiles and tries to build the projects\n"
    printf "Usage: chbuild.sh PATH\n"
    exit 1
fi

chbuild $1

exit $RETCODE
