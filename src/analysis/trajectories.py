import sys
import csv
from error import separateData, separateFollowerData, separateNeighborsData




def toFile(true_data, sim_data, input_type, metric):
    print('true_data len = ' + str(len(true_data)))
    print('sim_data len = ' + str(len(sim_data)))

    # Traj data for first 10 seconds of first simulated traj
    fn = "../../../trajectory_analysis_files/" + input_type + "_" + metric + ".csv"
    data = []
    for i in range(100):
        data.append([input_type, i, true_data[i], sim_data[i]])

    with open(fn, "w") as f:
        writer = csv.writer(f)
        writer.writerow(['type', 'timestep', 'true', 'sim'])
        writer.writerows(data)



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
        traj_dir += "/follower_01_basic_trajectories"
    elif input_type == "neighbors":
        traj_dir += "/neighbor_basic_trajectories"
        n = 21

    file_prefix = "/mixture_"
    file_suffix = ".csv"

    ### Output trajectory data for fold 1 ###
    datafile = traj_dir + file_prefix + '1' + file_suffix
    print("\nTrajectory analysis for fold 1")

    with open(datafile, "r") as f:
        sim_dist, sim_vel, sim_acc, true_dist, true_vel, true_acc = ([] for j in range(6))

        """ SEPARATE DATA """
        if input_type == "basic":
            separated_data = separateData(f)
            sim_dist, sim_vel, sim_acc, true_dist, true_vel, true_acc = separated_data

        elif input_type == "followers":
            separated_data = separateFollowerData(f)
            sim_dist, sim_vel, sim_acc, true_dist, true_vel, true_acc = separated_data

        elif input_type == "neighbors":
            separated_data = separateNeighborsData(f)
            sim_dist, sim_vel, sim_acc, true_dist, true_vel, true_acc = separated_data

        toFile(true_acc, sim_acc, input_type, "acc")
        toFile(true_vel, sim_vel, input_type, "vel")
        toFile(true_dist, sim_dist, input_type, "dist")


