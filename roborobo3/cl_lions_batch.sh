#PBS -N lions
#PBS -l walltime=12:00:00
#PBS -l nodes=1:ppn=24
#PBS -m n
#PBS -d /home/ecoffet/robocoop/roborobo3/roborobo3/


set -x  # Make the execution verbose
export SDL_VIDEODRIVER=dummy

# Variable definition

python=/home/ecoffet/anaconda3/bin/python

$python $dir/batch_lions.py $PBS_ARRAYID $time "$dir/lion.properties"
