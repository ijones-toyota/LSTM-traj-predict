import sys
import numpy as np
import matplotlib.pyplot as plt


"""
# Returns the mean and standard error for the given list of data
"""
def getMeanAndStdError(data):
    mean = 0
    for val in data:
        mean += val
    mean /= len(data)

    std_dev = 0
    for val in data:
        std_dev += np.power((val - mean), 2)

    std_dev = np.sqrt(std_dev / len(data))

    return [mean, std_dev / np.sqrt(len(data))]



"""
# Plots a dual bar chart across 10 folds for basic, follower, and neighbors data (rmse or negative state data)
# Data is aggregated across a 10-second prediction horizon for each fold
"""
def plotDualBarChart(fn, metric, var1Name, var2Name):

    var1 = {}
    var2 = {}

    # Read in data
    with open(fn, 'r') as f:
        i = 0
        for line in f:
            i += 1
            if i == 1:
                continue

            data = line.split(",")
            input_type = data[0]

            if input_type in var1:
                var1[input_type].append(float(data[2]))
            else:
                var1[input_type] = [float(data[2])]
            if input_type in var2:
                var2[input_type].append(float(data[3]))
            else:
                var2[input_type] = [float(data[3])]


    # VELOCITY ANALYTICS
    basic_vel_rmse, basic_vel_stderror = getMeanAndStdError(var1['basic'])
    follower_vel_rmse, follower_vel_stderror = getMeanAndStdError(var1['followers'])
    neighbors_vel_rmse, neighbors_vel_stderror = getMeanAndStdError(var1['neighbors'])

    var1_means = [basic_vel_rmse, follower_vel_rmse, neighbors_vel_rmse]
    var1_stderrors = [basic_vel_stderror, follower_vel_stderror, neighbors_vel_stderror]


    # ACCELERATION ANALYTICS
    basic_acc_rmse, basic_acc_stderror = getMeanAndStdError(var2['basic'])
    follower_acc_rmse, follower_acc_stderror = getMeanAndStdError(var2['followers'])
    neighbors_acc_rmse, neighbors_acc_stderror = getMeanAndStdError(var2['neighbors'])

    var2_means = [basic_acc_rmse, follower_acc_rmse, neighbors_acc_rmse]
    var2_stderrors = [basic_acc_stderror, follower_acc_stderror, neighbors_acc_stderror]


    # PLOT GRAPH
    n_groups = 3

    fig, ax = plt.subplots()

    index = np.arange(n_groups)
    bar_width = 0.2

    opacity = 0.4
    error_config = {'ecolor': '0.3'}

    var1Bar = ax.bar(index, var1_means, bar_width,
                    alpha=opacity, color="#4f4f4f",
                    yerr=var1_stderrors, error_kw=error_config,
                    label=var1Name)

    var2Bar = ax.bar(index + bar_width, var2_means, bar_width,
                    alpha=opacity, color='#b5b5b5',
                    yerr=var2_stderrors, error_kw=error_config,
                    label=var2Name)

    ax.set_ylabel(metric, labelpad=30)
    ax.set_xticks(index + bar_width)
    ax.set_xticklabels(('Basic', 'Follower', 'Neighbors'))
    # ax.set_xticklabels(('', '', ''))
    # ax.set_yticklabels((''))
    ax.tick_params(axis='x', pad=20)
    ax.tick_params(axis='y', pad=5)

    # Put a legend below current axis
    ax.legend()
    ax.legend(loc='upper center', bbox_to_anchor=(0.5, -0.075), ncol=2, frameon=False)

    fig.tight_layout()
    plt.grid()
    plt.show()



"""
# Plots a dual line chart across 10 folds for basic, follower, and neighbors data (rmse or negative state data)
# Data is split into 1-5 second prediction horizons for each fold
"""
def plotDualLineChart(fn, metric, varName):

    varIndex = -1   # Index in csv row of target variable
    if varName == 'Velocity':
        varIndex = 3
    elif varName == 'Acceleration':
        varIndex = 4

    var = {}

    # Read in data
    with open(fn, 'r') as f:
        i = 0
        for line in f:
            i += 1
            if i == 1:
                continue

            data = line.split(",")
            input_type = data[0]
            horizon = int(data[2]) - 1      # convert to 0-indexed

            if input_type in var:
                var[input_type][horizon].append(float(data[varIndex]))
            else:
                var[input_type] = [[] for j in range(5)]
                var[input_type][horizon] = [float(data[varIndex])]

    horizons = [1, 2, 3, 4, 5]
    basic_means = [0 for i in range(5)]
    follower_means = [0 for i in range(5)]
    neighbors_means = [0 for i in range(5)]

    basic_stderrors = [0 for i in range(5)]
    follower_stderrors = [0 for i in range(5)]
    neighbors_stderrors = [0 for i in range(5)]

    # Populate means and std errors at each horizon for each input network type
    for horizon in range(len(horizons)):
        basic_means[horizon], basic_stderrors[horizon] = getMeanAndStdError(var['basic'][horizon])
        follower_means[horizon], follower_stderrors[horizon] = getMeanAndStdError(var['followers'][horizon])
        neighbors_means[horizon], neighbors_stderrors[horizon] = getMeanAndStdError(var['neighbors'][horizon])

    horizons = [1, 2, 3, 4, 5]

    # PLOT GRAPH
    fig, ax = plt.subplots()

    basicLine = plt.plot(horizons, basic_means, label='Basic', linestyle='-', color='black')
    followerLine = plt.plot(horizons, follower_means, label='Follower', linestyle=":", color='black')
    neighborsLine = plt.plot(horizons, neighbors_means, label='Neighbors', linestyle="--", color='black')
    plt.errorbar(horizons, basic_means, yerr=basic_stderrors, fmt='o', color='black')
    plt.errorbar(horizons, follower_means, yerr=follower_stderrors, fmt='o', color='black')
    plt.errorbar(horizons, neighbors_means, yerr=neighbors_stderrors, fmt='o', color='black')

    # ax.set_xlabel('Prediction Horizon $(s)$', labelpad=15)
    ylabel = metric
    if varName == "Velocity":
        ylabel += " $(m/s)$"
    elif varName == "Acceleration":
        ylabel += " $(m/s^2)$"
    # ax.set_ylabel(ylabel, labelpad=15)
    plt.xticks(np.arange(min(horizons), max(horizons) + 1, 1.0))
    plt.ylim(0, 10)
    ax.tick_params(axis='x', pad=5)
    ax.tick_params(axis='y', pad=5)

    ax.set_xticklabels(('', '', ''))
    ax.set_yticklabels((''))

    # Put a legend below current axis
    ax.legend()
    ax.legend(loc='upper center', bbox_to_anchor=(0.5, -.1), ncol=3, frameon=False)

    fig.tight_layout()
    plt.grid()
    plt.show()



"""
# Plots a line chart for true, basic, follower, and neighbors trajectory data for a given 10 second prediction horizon
# Assumes csv file has columns: type, timestep, dataval
# type = 'true' | 'basic' | 'follower' | 'neighbors'
# timestep = 0-99
# dataval = vel | acc | headwaydist
"""
def plotTrajectories(fn, metric):

    true_data = [0 for i in range(100)]  # true trajectory
    sim = {}                             # simulated trajectories (basic, follower, neighbors)

    # Read in data
    with open(fn, 'r') as f:
        i = 0
        for line in f:
            i += 1
            if i == 1:
                continue

            data = line.split(",")
            input_type = data[0]
            # t = (float(data[1]) + 1) / 10.0
            t = int(data[1])

            # Add to true traj
            if input_type == "true":
                true_data[t] = float(data[2])

            # Add to simulated trajectories
            if input_type in sim:
                sim[input_type][t] = float(data[2])
            else:
                sim[input_type] = [0 for j in range(100)]
                sim[input_type][t] = float(data[2])


    timesteps = [(i / 10.0) for i in range(1, 101)]
    basic_data = sim['basic']
    follower_data = sim['follower']
    neighbors_data = sim['neighbors']

    # PLOT GRAPH
    fig, ax = plt.subplots()

    trueLine = plt.plot(timesteps, true_data, label='True Trajectory', linestyle="-", color='black')
    basicLine = plt.plot(timesteps, basic_data, label='Basic Simulated Trajectory', linestyle=":", color='black')
    followerLine = plt.plot(timesteps, follower_data, label='Follower Simulated Trajectory', linestyle="-.", color='black')
    neighborsLine = plt.plot(timesteps, neighbors_data, label='Neighbors Simulated Trajectory', linestyle="--", color='black')

    ax.set_xlabel('$Prediction$ $Horizon$ $(s)$', labelpad=15)
    ax.set_ylabel(metric, labelpad=15)
    plt.xticks(np.arange(0, 11, 1.0))
    ax.tick_params(axis='x', pad=5)
    ax.tick_params(axis='y', pad=5)

    # ax.set_xticklabels(('', '', ''))
    # ax.set_yticklabels((''))

    # Put a legend below current axis
    ax.legend()
    ax.legend(loc='upper center', bbox_to_anchor=(0.5, -.1), ncol=4, frameon=False)

    fig.tight_layout()
    plt.grid()
    plt.show()





""" 
# MAIN
"""
if __name__ == "__main__":
    SMALL_SIZE = 14
    MEDIUM_SIZE = 16
    BIGGER_SIZE = 30

    plt.rc('font', size=SMALL_SIZE)  # controls default text sizes
    plt.rc('axes', titlesize=BIGGER_SIZE)  # fontsize of the axes title
    plt.rc('axes', labelsize=BIGGER_SIZE)  # fontsize of the x and y labels
    plt.rc('xtick', labelsize=MEDIUM_SIZE)  # fontsize of the tick labels
    plt.rc('ytick', labelsize=MEDIUM_SIZE)  # fontsize of the tick labels
    plt.rc('legend', fontsize=SMALL_SIZE)  # legend fontsize
    plt.rcParams['figure.facecolor'] = 'white'
    plt.rcParams['savefig.facecolor'] = 'white'

    # plotDualBarChart('../../../analysis_files/combined-rmse.csv', "RMSE", "Velocity", "Acceleration")
    # plotDualBarChart('../../../analysis_files/combined-negatives.csv', "$Frequency$", "Negative Headway", "Negative Speed")
    # plotDualLineChart('../../../analysis_files/combined-horizons.csv', "RMSE", "Velocity")
    # plotDualLineChart('../../../analysis_files/combined-horizons.csv', "RMSE", "Acceleration")

    if len(sys.argv) > 1:
        traj = sys.argv[1]
        fn = '../../../trajectory_analysis_files/acc-time-' + traj + '.csv'
        plotTrajectories(fn, '$a(m/s^2)$')


