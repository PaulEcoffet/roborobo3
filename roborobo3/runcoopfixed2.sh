#clean
rm -rfI ../logs/coopfixed2_commonpot/
# CoopFixed2

mkdir -p ../logs/coopfixed2_commonpot//fake_rep_0/screenshots
nohup python pyevoroborobo.py -e cmaes -l config/CoopOpportunity2FixedFakeBots.properties -o ../logs/coopfixed2_commonpot//fake_rep_0/ -b > ../logs/coopfixed2_commonpot//fake_rep_0/stdout.txt 2> ../logs/coopfixed2_commonpot//fake_rep_0/stderr.txt &
sleep 1.5
mkdir -p ../logs/coopfixed2_commonpot//nofake_rep_0/screenshots
nohup python pyevoroborobo.py -e cmaes -l config/CoopOpportunity2Fixed.properties -o ../logs/coopfixed2_commonpot//nofake_rep_0/ -b > ../logs/coopfixed2_commonpot//nofake_rep_0/stdout.txt 2> ../logs/coopfixed2_commonpot//nofake_rep_0/stderr.txt &
sleep 1.5
mkdir -p ../logs/coopfixed2_commonpot//fake_rep_1/screenshots
nohup python pyevoroborobo.py -e cmaes -l config/CoopOpportunity2FixedFakeBots.properties -o ../logs/coopfixed2_commonpot//fake_rep_1/ -b > ../logs/coopfixed2_commonpot//fake_rep_1/stdout.txt 2> ../logs/coopfixed2_commonpot//fake_rep_1/stderr.txt &
sleep 1.5
mkdir -p ../logs/coopfixed2_commonpot//nofake_rep_1/screenshots
nohup python pyevoroborobo.py -e cmaes -l config/CoopOpportunity2Fixed.properties -o ../logs/coopfixed2_commonpot//nofake_rep_1/ -b > ../logs/coopfixed2_commonpot//nofake_rep_1/stdout.txt 2> ../logs/coopfixed2_commonpot//nofake_rep_1/stderr.txt &
sleep 1.5
mkdir -p ../logs/coopfixed2_commonpot//fake_rep_2/screenshots
nohup python pyevoroborobo.py -e cmaes -l config/CoopOpportunity2FixedFakeBots.properties -o ../logs/coopfixed2_commonpot//fake_rep_2/ -b > ../logs/coopfixed2_commonpot//fake_rep_2/stdout.txt 2> ../logs/coopfixed2_commonpot//fake_rep_2/stderr.txt &
sleep 1.5
mkdir -p ../logs/coopfixed2_commonpot//nofake_rep_2/screenshots
nohup python pyevoroborobo.py -e cmaes -l config/CoopOpportunity2Fixed.properties -o ../logs/coopfixed2_commonpot//nofake_rep_2/ -b > ../logs/coopfixed2_commonpot//nofake_rep_2/stdout.txt 2> ../logs/coopfixed2_commonpot//nofake_rep_2/stderr.txt &
sleep 1.5
wait

# wait until all simulations are over before analysis
wait

# Analysis

nohup ./roborobo -l config/CoopOpportunity2FixedAnalysis.properties -o ../logs/coopfixed2_commonpot//fake_rep_0/ -b > ../logs/coopfixed2_commonpot//fake_rep_0/stdout_analysis.txt 2> ../logs/coopfixed2_commonpot//fake_rep_0/stderr_analysis.txt &
sleep 1.5
nohup ./roborobo -l config/CoopOpportunity2FixedAnalysis.properties -o ../logs/coopfixed2_commonpot//nofake_rep_0/ -b > ../logs/coopfixed2_commonpot//nofake_rep_0/stdout_analysis.txt 2> ../logs/coopfixed2_commonpot//nofake_rep_0/stderr_analysis.txt &
sleep 1.5
nohup ./roborobo -l config/CoopOpportunity2FixedAnalysis.properties -o ../logs/coopfixed2_commonpot//fake_rep_1/ -b > ../logs/coopfixed2_commonpot//fake_rep_1/stdout_analysis.txt 2> ../logs/coopfixed2_commonpot//fake_rep_1/stderr_analysis.txt &
sleep 1.5
nohup ./roborobo -l config/CoopOpportunity2FixedAnalysis.properties -o ../logs/coopfixed2_commonpot//nofake_rep_1/ -b > ../logs/coopfixed2_commonpot//nofake_rep_1/stdout_analysis.txt 2> ../logs/coopfixed2_commonpot//nofake_rep_1/stderr_analysis.txt &
sleep 1.5
nohup ./roborobo -l config/CoopOpportunity2FixedAnalysis.properties -o ../logs/coopfixed2_commonpot//fake_rep_2/ -b > ../logs/coopfixed2_commonpot//fake_rep_2/stdout_analysis.txt 2> ../logs/coopfixed2_commonpot//fake_rep_2/stderr_analysis.txt &
sleep 1.5
nohup ./roborobo -l config/CoopOpportunity2FixedAnalysis.properties -o ../logs/coopfixed2_commonpot//nofake_rep_2/ -b > ../logs/coopfixed2_commonpot//nofake_rep_2/stdout_analysis.txt 2> ../logs/coopfixed2_commonpot//nofake_rep_2/stderr_analysis.txt &
sleep 1.5
wait

