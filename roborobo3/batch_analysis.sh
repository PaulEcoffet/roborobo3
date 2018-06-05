#!/bin/zsh
gen=$1
shift
for path in "$@"
do
  echo $path
  ./roborobo -l config/CoopOpportunity2FixedAnalysis.properties -o $path +genAnalysis $gen -b >> batch_log.txt 2>&1
done
