import sys
import csv
from error import separateData, separateFollowerData, separateNeighborsData






def getData(fn, input_type, metric):

    with open(fn, "r") as f:
        sim_dist, sim_vel, sim_acc, true_dist, true_vel, true_acc = ([] for j in range(6))

        """ SEPARATE DATA """
        if input_type == "basic":
            separated_data = separateData(f)
            sim_dist, sim_vel, sim_acc, true_dist, true_vel, true_acc = separated_data

            if metric == "acc":
                return [true_acc, sim_acc]
            if metric == "vel":
                return [true_vel, sim_vel]
            if metric == "dist":
                return [true_dist, sim_dist]

        elif input_type == "follower":
            separated_data = separateFollowerData(f)
            sim_dist, sim_vel, sim_acc, true_dist, true_vel, true_acc = separated_data

        elif input_type == "neighbors":
            separated_data = separateNeighborsData(f)
            sim_dist, sim_vel, sim_acc, true_dist, true_vel, true_acc = separated_data

        """ RETURN simulated data """
        if metric == "acc":
            return sim_acc
        elif metric == "vel":
            return sim_vel
        elif metric == "dist":
            return sim_dist




def toFile(true_data, basic_data, follower_data, neighbors_data, metric, start_time):

    # Traj data for first 10 seconds of first simulated traj
    fn = "../../../trajectory_analysis_files/" + metric + "-time-" + str(start_time) + ".csv"
    output = []

    m = start_time*100
    # true data
    for i in range(m, m + 100):
        output.append(["true", i % 100, true_data[i]])
    # sim basic data
    for i in range(m, m + 100):
        output.append(["basic", i % 100, basic_data[i*50]])
    # sim follower data
    for i in range(m, m + 100):
        output.append(["follower", i % 100, follower_data[i*50]])
    # sim neighbors data
    for i in range(m, m + 100):
        output.append(["neighbors", i % 100, neighbors_data[i*50]])

    with open(fn, "w") as f:
        writer = csv.writer(f)
        writer.writerow(['type', 'timestep', 'data'])
        writer.writerows(output)



""" 
# MAIN
# args: 
# argv[1] 'metric' = acc | vel | dist
"""
if __name__ == "__main__":
    metric = "acc"
    start_time = 0
    if len(sys.argv) > 1:
        metric     = sys.argv[1]        # acc | vel | dist
    if len(sys.argv) > 2:
        start_time = int(sys.argv[2])   # start time of trajectory to look at

    traj_dir = "/Users/ian/development/final/simulated_trajectories"
    file_prefix = "/mixture_"
    file_suffix = ".csv"

    ### Output trajectory data for fold 1 ###
    print("\nTrajectory analysis for fold 1")

    basic_file = traj_dir + "/basic_trajectories" + file_prefix + "1" + file_suffix
    true_data, basic_data = getData(basic_file, "basic", metric)

    follower_file = traj_dir + "/follower_01_basic_trajectories" + file_prefix + "1" + file_suffix
    follower_data = getData(follower_file, "follower", metric)

    neighbors_file = traj_dir + "/neighbor_basic_trajectories" + file_prefix + "1" + file_suffix
    neighbors_data = getData(neighbors_file, "neighbors", metric)


    toFile(true_data, basic_data, follower_data, neighbors_data, metric, start_time)


