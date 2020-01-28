#!/bin/bash


echo "create temporary dir"

dir=$(mktemp -d -t nego-XXXXXXXXX --tmpdir=/home/ecoffet/robocoop/roborobo3/roborobo3/batch_negociate_call)

echo "copying config files"

cp batch_negociate.py "$dir/batch_negociate.py"
cp config/negociate.properties "$dir/negociate.properties"
pythonexec='/home/ecoffet/anaconda3/bin/python'

nbjob=$($pythonexec "$dir/batch_negociate.py")

echo "submitting job"

jobid=`qsub -t 1-$nbjob -v dir="$dir,time=$(date +%s)" cl_negociate.sh`


echo "done"

