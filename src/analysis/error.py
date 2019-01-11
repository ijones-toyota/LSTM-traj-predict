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


    count = 0
    for a in true_vel:
        if a < 0:
            count += 1
            print(a)
    print("total negative true velocities = " + str(count))

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
# Calculates the Root Mean Squared Error of the velocity and acceleration predictions
"""
def calculateError(sim_vel, sim_acc, true_vel, true_acc):

    # Keep track of error
    vel_err, acc_err = (0,)*2

    # Iterate through all simulated trajectories
    for m in range(len(true_vel)):
        # Iterate through all samples in each simulated trajectory
        for n in range(50):
            vel_err += math.pow(sim_vel[(m * 50) + n] - true_vel[m], 2)
            acc_err += math.pow(sim_acc[(m * 50) + n] - true_acc[m], 2)

    vel_err  = math.sqrt(vel_err / len(sim_vel))
    acc_err  = math.sqrt(acc_err / len(sim_vel))

    print("Velocity Root Mean Squared Error     = " + str(vel_err))
    print("Acceleration Root Mean Squared Error = " + str(acc_err))



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
def countInversions(acc):

    j = jerk(acc)
    n = len(j)
    count = 0

    # loop over trajectories
    for i in range(0, n, 99):
        # loop over jerks in trajectory, increment count if sign change
        for k in range(i+1, i+99):
            if sign(j[k]) * sign(j[k-1]) < 0:
                count += 1

    return count * 99 / n       # average over all trajectories



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

    print("Frequency negative distance = " + str(num_neg_dist / len(sim_dist)))
    print("Frequency negative velocity = " + str(num_neg_vel / len(sim_vel)))




if __name__ == "__main__":
    input_type = sys.argv[1]
    traj_dir = "/Users/ian/development/final/simulated_trajectories"

    if input_type == "basic":
        traj_dir += "/basic_trajectories"
    elif input_type == "followers":
        traj_dir += "/follower_basic_trajectories"

    file_prefix = "/mixture_"
    file_suffix = ".csv"

    datafile = traj_dir + file_prefix + "1" + file_suffix
    print(datafile)

    with open(datafile, "r") as f:
        # Separated data
        if input_type == "basic":
            separated_data = separateData(f)
            sim_dist, sim_rel, sim_vel, sim_acc, true_dist, true_rel, true_vel, true_acc = separated_data

            calculateError(sim_vel, sim_acc, true_vel, true_acc)
            num_true_inversions = countInversions(true_acc)
            num_sim_inversions = countInversions(sim_acc)
            print("True inversions = " + str(num_true_inversions))
            print("Sim inversions  = " + str(num_sim_inversions))

            countNegatives(sim_dist, sim_vel, true_dist)

        elif input_type == "followers":
            separated_data = separateFollowerData(f)
            sim_dist, sim_rel, sim_vel, sim_acc, sim_follow_dist, sim_follow_rel, \
            true_dist, true_rel, true_vel, true_acc, true_follow_dist, true_follow_rel = separated_data

            calculateError(sim_vel, sim_acc, true_vel, true_acc)

            num_true_inversions = countInversions(true_acc)
            num_sim_inversions = countInversions(sim_acc)
            print("True inversions = " + str(num_true_inversions))
            print("Sim inversions  = " + str(num_sim_inversions))

            countNegatives(sim_dist, sim_vel, true_dist)





























