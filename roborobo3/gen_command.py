from os.path import join

i = 0
print("#clean")
print("rm -rf logs/coopfixed2_newfunc/")

print("# CoopFixed2")
print("")
template = {'nofake' :"python pyevoroborobo.py -e cmaes -l config/CoopOpportunity2Fixed.properties -o {} -b",
            'fake': "python pyevoroborobo.py -e cmaes -l config/CoopOpportunity2FixedFakeBots.properties -o {} -b"}
done = []
for rep in range(10):
    for cond in ['fake', 'nofake']:
        logpath = "logs/coopfixed2_newfunc/{0}_rep_{1}/".format(cond, rep)
        command = template[cond].format(logpath)
        full_command = "nohup {0} > {1}stdout.txt 2> {1}stderr.txt &".format(command, logpath)
        print("mkdir -p {}".format(join(logpath, 'screenshots')))
        print(full_command)
        print("sleep 1.5")
        done.append(logpath)
        i += 1
        if i == 6:
            print("wait")
            print("")
            i = 0

# Analysis
print("# wait until all simulations are over before analysis")
print("wait")
print("")
print("# Analysis")
print("")
template = "./roborobo -l config/CoopOpportunity2FixedAnalysis.properties -o {} -b"
for path in done:
    command = template.format(path)
    full_command = "nohup {0} > {1}stdout_analysis.txt 2> {1}stderr_analysis.txt &".format(command, logpath)
    print(full_command)
    print("sleep 1.5")
    i += 1
    if i == 6:
        print("wait")
        print("")
        i = 0
