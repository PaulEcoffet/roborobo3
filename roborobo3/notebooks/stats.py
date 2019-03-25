import os
import sys
import math
import pandas as pd
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.lines as mlines

# Parameters
exps = ['noavg-analysis']
nbGenomes = 8
generationTime = 2000
nbReplicas = 20
maxFakeRobots = 3
maxFakeCoop = 2.0

# Change working directory to the project
os.chdir(sys.argv[1])

for exp in exps:
    for genome in range(0, nbGenomes):
        print('Analyzing genome {} of exp {}\n'.format(genome, exp))

        # Read the stats file
        filename = '{}/coop_stats_{:02}.txt'.format(exp, genome)
        data = pd.read_csv(filename, delim_whitespace=True)

        # Add a payoff column: P(n, x0, x) = ...
        data['Payoff'] = np.where(data['nbRob'] == 0, 0,\
                np.sqrt(2*(data['Coop']+data['fkeRob']*data['fkeCoop']))\
                /(1+(data['nbRob']-2)**2)-data['Coop'])

        # Group the data by fkeCoop, fkeRob, and nbRob
        grouped_data = data.groupby(['fkeCoop', 'fkeRob', 'nbRob'])

        # Average over replicas and add the time spent in each case
        avg_data = grouped_data.mean().join(pd.DataFrame(grouped_data.size(),\
                                                         columns=['Time']))
        # Turn the Time column into a percentage
        avg_data['Time'] *= 100.0/(generationTime*nbReplicas)

        # For each fkeRobot level, plot Coop and Time over fakeCoop
        # We also remove rows spent away from objects by selecting rows
        # where nbRob == fkeRob+1 (otherwise nbRob == 0)

        # Figure with the robot effort value
        xlabel = 'Effort value per virtual robot'
        fig1, ax1 = plt.subplots()
        ax1.set_xlim([0, maxFakeCoop])
        ax1.set_ylim([0, 1])
        ax1.set_xlabel(xlabel)
        ax1.set_ylabel('Robot effort value')
        ax1nb = ax1.twinx()
        ax1nb.set_axis_off()

        # Figure with the robot participation time
        fig2, ax2 = plt.subplots()
        ax2.set_xlim([0, maxFakeCoop])
        ax2.set_ylim([0, 100]) # it's a percentage
        ax2.set_xlabel(xlabel)
        ax2.set_ylabel('Participation (% of time)')
        ax2nb = ax2.twinx()
        ax2nb.set_axis_off()

        # Figure with the robot payoff value
        fig3, ax3 = plt.subplots()
        ax3.set_xlim([0, maxFakeCoop])
        ax3.set_ylim([0, 2]) # for now
        ax3.set_xlabel(xlabel)
        ax3.set_ylabel('Robot payoff')
        ax3nb = ax3.twinx()
        ax3nb.set_axis_off()

        # Get some nice-ish colors
        cmap = matplotlib.cm.Dark2
        colors = [cmap(i) for i in np.linspace(0, 1, 8)]
        color_handles = []
        for nbRobots, color in zip(range(1, maxFakeRobots+1+1), colors):

            # Get the rows with that number of fake robots
            data_fkRob = avg_data.xs(nbRobots, level='nbRob')

            # Turn fkeCoop back into a column so that we can plot against it
            data_fkRob = data_fkRob.reset_index(level='fkeCoop')

            # Plot Coop on the first figure
            ax1.plot(data_fkRob['fkeCoop'], data_fkRob['Coop'], color=color)

            # Plot Time on the second figure
            ax2.plot(data_fkRob['fkeCoop'], data_fkRob['Time'], color=color)
            
            # Plot Payoff on the third figure
            ax3.plot(data_fkRob['fkeCoop'], data_fkRob['Payoff'], color=color)

            # Remember the color for the legend
            if nbRobots == 1:
                color_handles += [mlines.Line2D([], [], color=color, label='no virtual robots')]
            elif nbRobots == 2:
                color_handles += [mlines.Line2D([], [], color=color, label='1 virtual robot')]
            else:
                color_handles += [mlines.Line2D([], [], color=color, label='{} virtual robots'.format(nbRobots-1))]

        # Show the legends (number of virtual robots)
        ax1nb.legend(handles=color_handles, loc='best')
        ax2nb.legend(handles=color_handles, loc='best')
        ax3nb.legend(handles=color_handles, loc='best')

        # Save that into a file
        fig1.savefig('{}/gen{:02}_effort.png'.format(exp, genome, nbRobots-1))
        fig2.savefig('{}/gen{:02}_time.png'.format(exp, genome, nbRobots-1))
        fig3.savefig('{}/gen{:02}_payoff.png'.format(exp, genome, nbRobots-1))

        # Close the figures to avoid running out of memory
        plt.close(fig1)
        plt.close(fig2)
        plt.close(fig3)
