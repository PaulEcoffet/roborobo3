#!/bin/bash

echo "$@"

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

mkdir -p batch_log/
log=batch_log/log_$$.txt

shift

for path in "$@"
do
  echo "$i : $path"
  # time to launch
  if [ $findgen = 1 ]
  then
    lastfilename=`ls -tr ${path}/genomes*.txt | tail -n 1`
    real=`basename $lastfilename`
    gen=`echo $real | sed -e "s/[^0-9]//g"`
  fi

  echo "python playfromgenome.py -g $gen -p $path -o $path/replay +logEveryXGen 1 -b >> $log 2>&1 &"
  if [ $debug = true ]
  then
    echo 'debug mode'
    debugpath=`basename $path`
    python playfromgenome.py -g $gen -p $path -o $path/replay +logEveryXGen 1
    break
  elif [ $fake = true ]
  then
    if [[ -t 1 ]]  # Interactive shell
    then
      echo 'press enter to continue'
      read
    fi
    continue
  else
     python playfromgenome.py -g $gen -p $path -o $path/replay +logEveryXGen 1 -b >> $log 2>&1 &
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
