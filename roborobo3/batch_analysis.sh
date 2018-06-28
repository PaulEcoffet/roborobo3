#!/bin/bash


gen=$1
i=0

tmpdir=$(mktemp -d)
mkdir -p batch_log/
echo $$
log=batch_log/log_$$.txt

shift
for path in "$@"
do
  echo "$i : $path"
  # take the oldest conf found
  conf=$(/bin/ls -tr $path/*properties* | head -n 1)
  # copy it and remove useless import
  cp $conf $tmpdir/conf$i.properties
  sed -i '/import(.*/d' $tmpdir/conf$i.properties
  sed -i 's/gInitialNumberOfRobots.*/gInitialNumberOfRobots=1/' $tmpdir/conf$i.properties
  echo """
analysis = True
gPhysicalObjectDefaultType = 10 # Coop Fixed 2 Analysis object
analysisIterationPerRep=5000
analysisNbRep=3
""" >> $tmpdir/conf$i.properties
  # time to launch
  echo "./roborobo -l $tmpdir/conf$i.properties -o $path +genAnalysis $gen -b >> $log 2>&1 &"
  ./roborobo -l $tmpdir/conf$i.properties -o $path +genAnalysis $gen &

  # only run analyses 5 by 5
  let "i=$i+1"
  if [ "$i" = 5 ]
  then
    echo "wait"
    wait
    i=0
  fi
done
wait

# clean up

rm -r $tmpdir
