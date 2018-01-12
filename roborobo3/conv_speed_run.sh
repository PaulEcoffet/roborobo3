

# CoopFixed2

mkdir -p logs/coopfixed2/cmaes_rep_2/screenshots
nohup python pyevoroborobo.py -e cmaes -l config/CoopOpportunity2Fixed.properties -o logs/coopfixed2/cmaes_rep_2/ -b > logs/coopfixed2/cmaes_rep_2/stdout.txt 2> logs/coopfixed2/cmaes_rep_2/stderr.txt &
sleep 1.5
mkdir -p logs/coopfixed2/cmaes_rep_3/screenshots
nohup python pyevoroborobo.py -e cmaes -l config/CoopOpportunity2Fixed.properties -o logs/coopfixed2/cmaes_rep_3/ -b > logs/coopfixed2/cmaes_rep_3/stdout.txt 2> logs/coopfixed2/cmaes_rep_3/stderr.txt &
sleep 1.5
wait

