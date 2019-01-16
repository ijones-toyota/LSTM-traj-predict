import sys
import math



"""
# Separates the csv data into simulated and true vectors
# dist, rel, vel, acc for each category
# Params:
#   CSV file to read from, in the format outputted by cross_val.lua
# Returns:
#   List of lists
#   [0] simulated headway distance
#   [1] simulated relative velocity differential
#   [2] simulated velocity
#   [3] simulated acceleration
#   [4] true headway distance
#   [5] true relative velocity differential
#   [6] true velocity
#   [7] true acceleration
"""
def separateData(dataset):

    print "separating data..."

    sim_dist = []
    sim_rel  = []
    sim_vel  = []
    sim_acc  = []

    true_dist = []
    true_rel  = []
    true_vel  = []
    true_acc  = []

    for line in dataset:
        line = line.split(',')
        true_dist.append(float(line[200]))
        true_rel.append(float(line[201]))
        true_vel.append(float(line[202]))
        true_acc.append(float(line[203]))

        # Add list to keep track of samples for this trajectory

        # Loop over simulated values for this timestep
        for i in range(0, len(line) - 4):
            # Headway distance
            if i % 4 == 0:
                sim_dist.append(float(line[i]))

            # Relative speed differential
            elif (i-1) % 4 == 0:
                sim_rel.append(float(line[i]))

            # Velocity
            elif (i-2) % 4 == 0:
                sim_vel.append(float(line[i]))

            # Acceleration
            elif (i-3) % 4 == 0:
                sim_acc.append(float(line[i]))

    return sim_dist, sim_rel, sim_vel, sim_acc, true_dist, true_rel, true_vel, true_acc



"""
# Separates the csv data into simulated and true vectors
# dist, rel, vel, acc for each category
# Params:
#   CSV file to read from, in the format outputted by cross_val.lua
# Returns:
#   List of lists
#   [0] simulated headway distance
#   [1] simulated relative velocity differential
#   [2] simulated velocity
#   [3] simulated acceleration
#   [4] simulated follower headway distance
#   [5] simulated follower velocity differential
#   [6] true headway distance
#   [7] true relative velocity differential
#   [8] true velocity
#   [9] true acceleration
#   [10] true follow headway distance
#   [11] true follow relative velocity differential
"""
def separateFollowerData(dataset):

    print "separating data..."

    sim_dist = []
    sim_rel  = []
    sim_vel  = []
    sim_acc  = []
    sim_follow_dist = []
    sim_follow_rel  = []

    true_dist = []
    true_rel  = []
    true_vel  = []
    true_acc  = []
    true_follow_dist = []
    true_follow_rel  = []

    for line in dataset:
        line = line.split(',')
        true_dist.append(float(line[300]))
        true_rel.append(float(line[301]))
        true_vel.append(float(line[302]))
        true_acc.append(float(line[303]))
        true_follow_dist.append(float(line[304]))
        true_follow_rel.append(float(line[305]))

        # Add list to keep track of samples for this trajectory

        # Loop over simulated values for this timestep
        for i in range(0, len(line) - 6):
            # Leader headway distance
            if i % 6 == 0:
                sim_dist.append(float(line[i]))

            # Leader relative speed differential
            elif (i-1) % 6 == 0:
                sim_rel.append(float(line[i]))

            # Velocity
            elif (i-2) % 6 == 0:
                sim_vel.append(float(line[i]))

            # Acceleration
            elif (i-3) % 6 == 0:
                sim_acc.append(float(line[i]))

            # Follower headway distance
            elif (i-4) % 6 == 0:
                sim_follow_dist.append(float(line[i]))

            # Follower relative speed differential
            elif (i-5) % 6 == 0:
                sim_follow_rel.append(float(line[i]))

    count = 0
    for a in true_vel:
        if a < 0:
            count += 1
            print(a)
    print("total negative true velocities = " + str(count))

    return sim_dist, sim_rel, sim_vel, sim_acc, sim_follow_dist, sim_follow_rel, true_dist, true_rel, true_vel, true_acc, true_follow_dist, true_follow_rel



"""
# Separates the csv data into simulated and true vectors
# dist, rel, vel, acc for each category
# Params:
#   CSV file to read from, in the format outputted by cross_val.lua
# Returns:
#   List of lists
#   [0] simulated headway distance
#   [1] simulated relative velocity differential
#   [2] simulated velocity
#   [3] simulated acceleration
#   [4] simulated follower headway distance
#   [5] simulated follower velocity differential
#   [6] true headway distance
#   [7] true relative velocity differential
#   [8] true velocity
#   [9] true acceleration
#   [10] true follow headway distance
#   [11] true follow relative velocity differential
"""
def separateNeighborsData(dataset):

    print "separating data..."

    sim_dist = []
    sim_vel  = []
    sim_acc  = []

    true_dist = []
    true_vel  = []
    true_acc  = []

    for line in dataset:
        line = line.split(',')
        true_vel.append(float(line[900]))
        true_acc.append(float(line[901]))
        true_dist.append(float(line[902]))

        # Add list to keep track of samples for this trajectory

        # Loop over simulated values for this timestep
        for i in range(0, len(line) - 18):
            # Leader headway distance
            if i % 18 == 0:
                sim_vel.append(float(line[i]))

            # Velocity
            elif (i-1) % 18 == 0:
                sim_acc.append(float(line[i]))

            # Acceleration
            elif (i-2) % 18 == 0:
                sim_dist.append(float(line[i]))

    count = 0
    for a in true_vel:
        if a < 0:
            count += 1
            print(a)
    print("total negative true velocities = " + str(count))

    return sim_dist, sim_vel, sim_acc, true_dist, true_vel, true_acc



"""
# Calculates the Mean Root Squared Error of the velocity and acceleration predictions
"""
def calculateError(sim_vel, sim_acc, true_vel, true_acc):

    # Keep track of error
    vel_err, acc_err = (0,)*2
    # Keep track of error at 1 to 5 second horizons
    hor_vel_err = [0 for i in range(5)]
    hor_acc_err = [0 for i in range(5)]
    # Keep track of total samples that each horizon takes into account
    total_hor_samples = [0 for i in range(5)]


    # Iterate through all simulated trajectories
    for m in range(len(true_vel)):

        # Determine time horizon for this trajectory
        horizon = m % 100
        horizon = horizon / 10
        # Don't care about horizons past 5 seconds
        if horizon > 4:
            horizon = -1

        # Iterate through all samples in each simulated trajectory
        for n in range(50):
            # Error for this sample
            sample_vel_err = math.pow(sim_vel[(m * 50) + n] - true_vel[m], 2)
            sample_acc_err = math.pow(sim_acc[(m * 50) + n] - true_acc[m], 2)

            # Add to total err
            vel_err += sample_vel_err
            acc_err += sample_acc_err

            # Add to horizon error calculation
            if horizon != -1:
                if horizon <= 0:
                    hor_vel_err[0] += sample_vel_err
                    hor_acc_err[0] += sample_acc_err
                    total_hor_samples[0] += 1

                if horizon <= 1:
                    hor_vel_err[1] += sample_vel_err
                    hor_acc_err[1] += sample_acc_err
                    total_hor_samples[1] += 1

                if horizon <= 2:
                    hor_vel_err[2] += sample_vel_err
                    hor_acc_err[2] += sample_acc_err
                    total_hor_samples[2] += 1

                if horizon <= 3:
                    hor_vel_err[3] += sample_vel_err
                    hor_acc_err[3] += sample_acc_err
                    total_hor_samples[3] += 1

                if horizon <= 4:
                    hor_vel_err[4] += sample_vel_err
                    hor_acc_err[4] += sample_acc_err
                    total_hor_samples[4] += 1



    mrse_vel  = math.sqrt(1.0 * vel_err / len(sim_vel))
    mrse_acc  = math.sqrt(1.0 * acc_err / len(sim_vel))
    print("Velocity Mean Root Squared Error     = %.4f" % mrse_vel)
    print("Acceleration Mean Root Squared Error = %.4f" % mrse_acc)

    print("MRSE subdivided by 1-5 sec time horizon")
    for i in range(len(hor_vel_err)):
        hor_vel_err[i] /= (1.0 * total_hor_samples[i])
        hor_acc_err[i] /= (1.0 * total_hor_samples[i])
        print("horizon " + str(i+1))
        print("Velocity MRSE     = %.4f" % hor_vel_err[i])
        print("Acceleration MRSE = %.4f" % hor_acc_err[i])

    return [mrse_vel, mrse_acc, hor_vel_err, hor_acc_err]



"""
# Stores the jerk values for each timestep in each acceleration trajectory and returns list
"""
def jerk(acc):

    # total number of trajectories
    n = len(acc) / 100

    # vector to hold jerk values
    j = [0 for i in range(99*n)]

    # loop over trajectories
    for i in range(n):
        # loop over timesteps in trajectory, store jerk
        for k in range(1, 100):
            j[i * 99 + k - 1] = (acc[i * 100 + k] - acc[i * 100 + k - 1]) / 0.1

    return j



"""
# Inline for fast inversion calculations
"""
def sign(x):
    if x > 0:
        return 1
    elif x < 0:
        return -1
    else:
        return x



"""
# Loops over all acceleration trajectories and counts the number of jerk inversions
"""
def countInversions(acc, type):

    j = jerk(acc)
    n = len(j)
    count = 0

    # loop over trajectories
    for i in range(0, n, 99):
        # loop over jerks in trajectory, increment count if sign change
        for k in range(i+1, i+99):
            if sign(j[k]) * sign(j[k-1]) < 0:
                count += 1

    if type == "True":
        print("True inversions      = " + str(count * 99 / n))
    elif type == "Simulated":
        print("Simulated inversions = " + str(count * 99 / n))

    return [count * 99, n]



"""
# Loops over all distance and velocity trajectories and counts the number of negative speed and distance headway values
"""
def countNegatives(sim_dist, sim_vel, true_dist):
    num_neg_dist = 0.0

    for i, elem in enumerate(sim_dist):
        # Only take into account negative distance headways when there is a leading vehicle
        if elem < 0 and true_dist[i/50] != 0:
            num_neg_dist += 1

    num_neg_vel = 0.0
    for elem in sim_vel:
        if elem < 0:
            num_neg_vel += 1

    print("Raw number negative distance = " + str(num_neg_dist))
    print("Raw number negative velocity = " + str(num_neg_vel))

    print("Frequency negative distance = %.4f" % (num_neg_dist / len(sim_dist)))
    print("Frequency negative velocity = %.4f" % (num_neg_vel / len(sim_vel)))

    return [num_neg_dist, num_neg_vel]



"""
# Prints out the error, inversions, and negative calculations
"""
def printAnalysis(n_folds, vel_err, acc_err, hor_vel_err, hor_acc_err, num_true_inversions, total_true_inversion_traj,
                  num_sim_inversions, total_sim_inversion_traj, num_neg_dist, num_neg_vel, total_sim_traj):

    print("\n\n")
    print("Analysis across all 10 folds:")

    # Display error across all folds
    mrse_vel = vel_err / n_folds
    mrse_acc = acc_err / n_folds
    print("Average Velocity MRSE     = %.4f" % mrse_vel)
    print("Average Acceleration MRSE = %.4f" % mrse_acc)

    # Display error for each time horizon
    mrse_hor1_vel = hor_vel_err[0] / n_folds
    mrse_hor1_acc = hor_acc_err[0] / n_folds
    mrse_hor2_vel = hor_vel_err[1] / n_folds
    mrse_hor2_acc = hor_acc_err[1] / n_folds
    mrse_hor3_vel = hor_vel_err[2] / n_folds
    mrse_hor3_acc = hor_acc_err[2] / n_folds
    mrse_hor4_vel = hor_vel_err[3] / n_folds
    mrse_hor4_acc = hor_acc_err[3] / n_folds
    mrse_hor5_vel = hor_vel_err[4] / n_folds
    mrse_hor5_acc = hor_acc_err[4] / n_folds
    print("Avg MRSE subdivided by 1-5 sec time horizon:")
    print("1 second horizon:")
    print("Avg Velocity MRSE     = %.4f" % mrse_hor1_vel)
    print("Avg Acceleration MRSE = %.4f" % mrse_hor1_acc)
    print("2 second horizon:")
    print("Avg Velocity MRSE     = %.4f" % mrse_hor2_vel)
    print("Avg Acceleration MRSE = %.4f" % mrse_hor2_acc)
    print("3 second horizon:")
    print("Avg Velocity MRSE     = %.4f" % mrse_hor3_vel)
    print("Avg Acceleration MRSE = %.4f" % mrse_hor3_acc)
    print("4 second horizon:")
    print("Avg Velocity MRSE     = %.4f" % mrse_hor4_vel)
    print("Avg Acceleration MRSE = %.4f" % mrse_hor4_acc)
    print("5 second horizon:")
    print("Avg Velocity MRSE     = %.4f" % mrse_hor5_vel)
    print("Avg Acceleration MRSE = %.4f" % mrse_hor5_acc)

    # Display inversions across all folds
    avg_true_inversions = 1.0 * num_true_inversions / total_true_inversion_traj
    avg_sim_inversions = 1.0 * num_sim_inversions / total_sim_inversion_traj
    print("Avg True Jerk Inversions      = %.4f" % avg_true_inversions)
    print("Avg Simulated Jerk Inversions = %.4f" % avg_sim_inversions)

    # Display negative values across all folds
    avg_neg_dist = 1.0 * num_neg_dist / total_sim_traj
    avg_neg_vel = 1.0 * num_neg_vel / total_sim_traj
    print("Avg Total Negative Headway Distances = %.4f" % avg_neg_dist)
    print("Avg Total Negative Velocities        = %.4f" % avg_neg_vel)



""" 
# MAIN
"""
if __name__ == "__main__":
    input_type = sys.argv[1]
    # number of trajectory files to iterate through +1
    n = 11

    traj_dir = "/Users/ian/development/final/simulated_trajectories"

    if input_type == "basic":
        traj_dir += "/basic_trajectories"
    elif input_type == "followers":
        traj_dir += "/follower_02_basic_trajectories"
    elif input_type == "neighbors":
        traj_dir += "/neighbor_basic_trajectories"
        n = 21

    file_prefix = "/mixture_"
    file_suffix = ".csv"


    # Total simulated trajectories
    total_sim_traj = 0

    # Error
    vel_err = 0
    acc_err = 0

    # Keep track of error at 1 to 5 second horizons
    hor_vel_err = [0 for i in range(5)]
    hor_acc_err = [0 for i in range(5)]

    # Inversions
    num_true_inversions = 0
    num_sim_inversions = 0
    total_true_inversion_traj = 0
    total_sim_inversion_traj = 0


    # Negatives
    num_neg_dist = 0
    num_neg_vel = 0


    ### Loop through 10 validation sets ###
    for i in range(1, n):
        datafile = traj_dir + file_prefix + str(i) + file_suffix
        print("\nAnalysis for fold " + str(i))

        with open(datafile, "r") as f:
            sim_dist, sim_vel, sim_acc, true_dist, true_vel, true_acc = ([] for j in range(6))

            """ SEPARATE DATA """
            if input_type == "basic":
                separated_data = separateData(f)
                sim_dist, sim_rel, sim_vel, sim_acc, true_dist, true_rel, true_vel, true_acc = separated_data

            elif input_type == "followers":
                separated_data = separateFollowerData(f)
                sim_dist, sim_rel, sim_vel, sim_acc, sim_follow_dist, sim_follow_rel, \
                true_dist, true_rel, true_vel, true_acc, true_follow_dist, true_follow_rel = separated_data

            elif input_type == "neighbors":
                separated_data = separateNeighborsData(f)
                sim_dist, sim_vel, sim_acc, true_dist, true_vel, true_acc = separated_data


            # Keep track of total simulated trajectories
            total_sim_traj += len(sim_vel)

            """ ERROR """
            # Keep track of MRSE before averaging for velocity and acceleration
            error_vals = calculateError(sim_vel, sim_acc, true_vel, true_acc)
            vel_err += error_vals[0]
            acc_err += error_vals[1]
            # Keep track of MRSE before averaging for each 1-5 sec time horizon
            for j in range(len(hor_vel_err)):
                hor_vel_err[j] += error_vals[2][j]
                hor_acc_err[j] += error_vals[3][j]

            """ JERK CALCULATIONS """
            # Keep track of true jerk inversions
            inversion_vals = countInversions(true_acc, "True")
            num_true_inversions += inversion_vals[0]
            total_true_inversion_traj += inversion_vals[1]

            # Keep track of simulated jerk inversions
            inversion_vals = countInversions(sim_acc, "Simulated")
            num_sim_inversions += inversion_vals[0]
            total_sim_inversion_traj += inversion_vals[1]

            """ NEGATIVE HEADWAY DISTANCE / VELOCITY """
            negative_vals = countNegatives(sim_dist, sim_vel, true_dist)
            num_neg_dist += negative_vals[0]
            num_neg_vel += negative_vals[1]


    """ Print statistics """
    printAnalysis(n-1, vel_err, acc_err, hor_vel_err, hor_acc_err, num_true_inversions, total_true_inversion_traj,
                  num_sim_inversions, total_sim_inversion_traj, num_neg_dist, num_neg_vel, total_sim_traj)


















