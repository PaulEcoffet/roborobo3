#PBS -N partnercontrol_bigmodel
#PBS -l walltime=160:00:00
#PBS -l nodes=1:ppn=32
#PBS -m n
#PBS -d /home/ecoffet/robocoop/roborobo3/roborobo3/

export SDL_VIDEODRIVER=dummy

# Variable definition

python=/home/ecoffet/.virtualenvs/robocoop/bin/python

groupname=`date +%F-%H%M`"-bm-rep-big-fitprop"
logdir="/home/ecoffet/robocoop/logs/$groupname"
nb_rep=5
confs=( 'bm_rep_big/other_noinv' 'bm_rep_big/other_noinv_fake')
sigma=0.001

let "nbsplit=32 / (${#confs[@]} * nb_rep)"

for curconf in "${confs[@]}"
do
    echo "starting $curconf"
    name=`basename $curconf`
    for i in `seq -w $nb_rep`
    do
        $python pyevoroborobo.py -e fitprop -s $sigma -l "config/${curconf}.properties" -o $logdir/$name/run_$i/ -p $nbsplit -b > $logdir/out.txt 2> $logdir/err.txt &
    done
done

wait
echo "Over"
