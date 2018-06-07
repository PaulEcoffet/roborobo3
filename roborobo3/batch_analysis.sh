#!/bin/zsh
gen=$1
i=0
shift
for path in "$@"
do
  echo "$i : $path"
  ./roborobo -l config/CoopOpportunity2FixedAnalysis.properties -o $path +genAnalysis $gen -b >> batch_log.txt 2>&1 &
  let "i=$i+1"
  if [ "$i" = 3 ]
  then
    wait
    i=0
  fi
done
wait
