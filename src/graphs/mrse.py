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
# Plots a dual bar chart across 10 folds for basic, follower, and neighbors data (mrse or negative state data)
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
    basic_vel_mrse, basic_vel_stderror = getMeanAndStdError(var1['basic'])
    follower_vel_mrse, follower_vel_stderror = getMeanAndStdError(var1['followers'])
    neighbors_vel_mrse, neighbors_vel_stderror = getMeanAndStdError(var1['neighbors'])

    var1_means = [basic_vel_mrse, follower_vel_mrse, neighbors_vel_mrse]
    var1_stderrors = [basic_vel_stderror, follower_vel_stderror, neighbors_vel_stderror]


    # ACCELERATION ANALYTICS
    basic_acc_mrse, basic_acc_stderror = getMeanAndStdError(var2['basic'])
    follower_acc_mrse, follower_acc_stderror = getMeanAndStdError(var2['followers'])
    neighbors_acc_mrse, neighbors_acc_stderror = getMeanAndStdError(var2['neighbors'])

    var2_means = [basic_acc_mrse, follower_acc_mrse, neighbors_acc_mrse]
    var2_stderrors = [basic_acc_stderror, follower_acc_stderror, neighbors_acc_stderror]


    # PLOT GRAPH
    n_groups = 3

    fig, ax = plt.subplots()

    index = np.arange(n_groups)
    bar_width = 0.35

    opacity = 0.4
    error_config = {'ecolor': '0.3'}

    var1Bar = ax.bar(index, var1_means, bar_width,
                    alpha=opacity, color='b',
                    yerr=var1_stderrors, error_kw=error_config,
                    label=var1Name)

    var2Bar = ax.bar(index + bar_width, var2_means, bar_width,
                    alpha=opacity, color='r',
                    yerr=var2_stderrors, error_kw=error_config,
                    label=var2Name)

    ax.set_xlabel('Network')
    ax.set_ylabel(metric)
    ax.set_title('Mean Root Squared Error across 10 folds')
    ax.set_xticks(index + bar_width)
    ax.set_xticklabels(('Basic', 'Follower', 'Neighbors'))
    ax.legend()

    fig.tight_layout()
    plt.show()


"""
# Plots a dual line chart across 10 folds for basic, follower, and neighbors data (mrse or negative state data)
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

    basicLine = plt.plot(horizons, basic_means, label='Basic')
    followerLine = plt.plot(horizons, follower_means, label='Follower')
    neighborsLine = plt.plot(horizons, neighbors_means, label='Neighbors')
    plt.errorbar(horizons, basic_means, yerr=basic_stderrors, fmt='o')
    plt.errorbar(horizons, follower_means, yerr=follower_stderrors, fmt='o')
    plt.errorbar(horizons, neighbors_means, yerr=neighbors_stderrors, fmt='o')

    ax.set_xlabel('Prediction Horizon (s)')
    ax.set_ylabel(metric)
    ax.set_title('Error in predictions over different prediction horizons')
    plt.xticks(np.arange(min(horizons), max(horizons) + 1, 1.0))
    ax.legend()

    fig.tight_layout()
    plt.show()




""" 
# MAIN
"""
if __name__ == "__main__":

    # plotDualBarChart('../../../analysis_files/combined-mrse.csv', "MRSE", "Velocity", "Acceleration")
    # plotDualBarChart('../../../analysis_files/combined-negatives.csv', "Frequency", "Negative Headway", "Negative Speed")
    # plotDualLineChart('../../../analysis_files/combined-horizons.csv', "MRSE", "Velocity")










