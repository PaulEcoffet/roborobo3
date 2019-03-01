#!/bin/bash

debug=false
fake=false
max=10
if [ $1 = "-d" ]
then
  debug=true
  shift
elif [ $1 = "-f" ]
then
  fake=true
  shift
elif [ $1 = "-p" ]
then
  shift
  max=$1
  shift
fi

gen=$1
i=0

if [ $gen = 0 ]
then
    findgen=1
else
    findgen=0
fi


echo $debug

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
  sed -i.bak '/import(.*/d' "$tmpdir/conf$i.properties"
  sed -i.bak 's/gRandomSeed.*/gRandomSeed=-1/' "$tmpdir/conf$i.properties"
  sed -i.bak 's/gInitialNumberOfRobots.*/gInitialNumberOfRobots=1/' "$tmpdir/conf$i.properties"
  echo """
analysis = True
gPhysicalObjectDefaultType = 10 # Coop Fixed 2 Analysis object
analysisIterationPerRep=1
analysisNbRep=1
""" >> $tmpdir/conf$i.properties

  # time to launch
  if [ $findgen = 1 ]
  then
    lastfilename=`ls -tr ${path}/genomes*.txt | tail -n 1`
    real=`basename $lastfilename`
    gen=`echo $real | sed -e "s/[^0-9]//g"`
    echo $gen
  fi

  echo "./roborobo -l $tmpdir/conf$i.properties -b -o $path +genAnalysis $gen >> $log 2>&1 &"
  if [ $debug = true ]
  then
    echo 'debug mode'
    debugpath=`basename $path`
    ./roborobo -l $tmpdir/conf$i.properties -o $path +genAnalysis $gen
    break
  elif [ $fake = true ]
  then
      echo 'press enter to continue'
      read
    continue
  else
    ./roborobo -l $tmpdir/conf$i.properties -b -o $path +genAnalysis $gen  >> $log 2>&1 &
  fi

  # only run analyses 10 by 10
  let "i=$i+1"
  if [ "$i" = $max ]
  then
    echo "wait"
    wait
    i=0
  fi
done
wait

# clean up

rm -r $tmpdir
