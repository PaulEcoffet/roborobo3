#!/bin/bash


echo "create temporary dir"


dir=$(mktemp -d -t lions-XXXXXXXXX --tmpdir=/home/ecoffet/robocoop/roborobo3/roborobo3/_batch_lions_call)

echo "copying config files"

cp batch_lions.py "$dir/batch_lions.py"
cp config/lion_megabig.properties "$dir/lion.properties"
pythonexec='/home/ecoffet/anaconda3/bin/python'

nbjob=$($pythonexec "$dir/batch_lions.py")

if [ -n "$1" ]
then
    nbjob="$1"
fi

cur=1
targ=$(( 200 < nbjob ? 200 : nbjob))

while (( $cur <= $nbjob ))
do
    echo "submitting job from $cur to $targ"
    jobid=`qsub -t $cur-$targ -v dir="$dir,time=$(date +%s)" cl_lions_batch.sh`
    let "cur=targ+1"
    let "targ=cur+200"
    targ=$(( targ < nbjob ? targ : nbjob ))
done

echo "done"
