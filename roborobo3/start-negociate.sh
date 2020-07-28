#!/bin/bash


echo "create temporary dir"

dir=$(mktemp -d -t nego-XXXXXXXXX --tmpdir=/home/ecoffet/robocoop/roborobo3/roborobo3/batch_negociate_call)

echo "copying config files"

cp batch_negociate.py "$dir/batch_negociate.py"
cp config/negociate.properties "$dir/negociate.properties"
pythonexec='/home/ecoffet/anaconda3/bin/python'

nbjob=$($pythonexec "$dir/batch_negociate.py")

cur=1
targ=$(( 200 < nbjob ? 200 : nbjob))
while (( $cur <= $nbjob ))
do
    echo "submitting job from $cur to $targ"
    jobid=`qsub -t $cur-$targ -v dir="$dir,time=$(date +%s)" cl_negociate.sh`
    let "cur=targ+1"
    let "targ=cur+200"
    targ=$(( targ < nbjob ? targ : nbjob ))
done

echo "done"

