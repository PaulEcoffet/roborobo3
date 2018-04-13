#PBS -N partnercontrol_bigmodel
#PBS -l walltime=120:00:00
#PBS -l nodes=1:ppn=32
#PBS -m n
#PBS -d /home/ecoffet/robocoop/roborobo3/roborobo3/

export SDL_VIDEODRIVER=dummy

# Variable definition

python=/home/ecoffet/.virtualenvs/robocoop/bin/python

groupname=`date +%F-%H%M`"-bm-grid-inv-input"
logdir="/home/ecoffet/robocoop/logs/$groupname"
nb_rep=5
confs=( 'bm_inv/notot_noinv' 'bm_inv/other_inv' 'bm_inv/other_noinv'
        'bm_inv/tot_inv' 'bm_inv/tot_noinv' 'bm_inv/notot_inv')

let "nbsplit=32 / (${#confs[@]} * nb_rep)"

for curconf in "${confs[@]}"
do
    echo "starting $curconf"
    name=`basename $curconf`
    for i in `seq -w $nb_rep`
    do
        $python pyevoroborobo.py -e cmaes -l "config/${curconf}.properties" -o $logdir/$name/run_$i/ -p $nbsplit -b&
    done
done

wait
echo "Over"
