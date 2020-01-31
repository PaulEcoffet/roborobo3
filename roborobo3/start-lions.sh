#!/bin/bash


echo "create temporary dir"

mkdir /home/ecoffet/robocoop/roborobo3/roborobo3/_batch_lions_call || true # ignore error

dir=$(mktemp -d -t lions-XXXXXXXXX --tmpdir=/home/ecoffet/robocoop/roborobo3/roborobo3/_batch_lions_call)

echo "copying config files"

cp batch_lions.py "$dir/batch_lions.py"
cp config/lion_megabig.properties "$dir/lion.properties"
pythonexec='/home/ecoffet/anaconda3/bin/python'

nbjob=$($pythonexec "$dir/batch_lions.py")

echo "submitting job"

jobid=`qsub -t 1-$nbjob -v dir="$dir,time=$(date +%s)" cl_lions_batch.sh`


echo "done"
