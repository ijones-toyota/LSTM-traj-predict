import sys
import csv



"""
# Filter through the dataset, organizing by vehicle id and timestep (frame)
# Params:
#   NGSIM dataset
# Returns:
#   Dictionary {Vehicle ID : {timestep : [list of vehicle info]}}
#   Dictionary where vehicle ids are mapped to dictionaries that map timesteps to vehicle information
# List of vehicle info: [class, v_Vel, v_Acc, Space_Headway, Preceding, Following, Lane_ID, v_y]
#   [0] v_Vel         = ego vehicle's velocity at time t
#   [1] v_Acc         = ego vehicle's acceleration at time t
#   [2] Lane_ID       = ego vehicle's lane ID, filter out data from lanes that don't exits (bad data)
#   [3] v_y           = ego vehicle's y position
#   [4] Leader        = Vehicle ID of lead vehicle in same lane
#   [5] Follower      = Vehicle ID of trailing vehicle in same lane
#
"""
def filterByVehicleID(dataset):
    print "filtering by vehicle id..."
    vehicle_info = {}
    invalidIDCount = 0
    # Iterate through dataset #
    for line in dataset:
        line = line.split(',')
        id        = int(line[0].strip())        # Current vehicle ID
        frame     = int(line[1].strip())        # Current frame
        local_y   = float(line[5].strip())      # Y position
        v_vel     = float(line[11].strip())     # Ego velocity
        v_acc     = float(line[12].strip())     # Ego acceleration
        lane_id   = int(line[13].strip())       # Lane
        leader    = int(line[20].strip())       # ID of vehicle ahead of ego
        follower  = int(line[21].strip())       # ID of vehicle behind ego
        if lane_id == 0 or lane_id > 8: # bad data
            # print('invalid lane_id for vehicle ' + str(id) + ' at frame ' + str(frame) + ' (lane_id = ' + str(lane_id) + ')')
            invalidIDCount += 1
            continue

        info = [v_vel, v_acc, lane_id, local_y, leader, follower]

        # Current vehicle id in dict
        if id in vehicle_info:
            vehicle_info[id][frame] = info

        # Current vehicle id not in dict, add to dict
        else:
            vehicle_info[id] = {}
            vehicle_info[id][frame] = info

    print(str(invalidIDCount) + " invalid Lane Id's recorded.")
    return vehicle_info



"""
# Iterate through all timesteps for each vehicle, compiling the neighboring vehicle information
# Neighboring vehicles are considered for the ego's lane and the lane directly to the left and right of the ego
# In the left and right lanes, 3 vehicles are considered: top left/right, left/right, and bottom left/right
# Params:
#   Dictionary where {Vehicle ID : {timestep : [list of vehicle info]}}
#   ^^ Return value from filterByVehicleID() method
# Returns:
#   Dictionary that maps vehicle ids to a list of their vehicle info at each timestep
#   Dictionary where {Vehicle ID : [list of [list of vehicle info]]
# List of vehicle info:
# X VALUES
#   [0]  v_t  =  ego velocity at timestep t 
#   [1]  a_t  =  ego acceleration at timestep t
#
#   [2]  dl_t  =  leader/ego distance between at timestep t 
#   [3]  rl_t  =  leader/ego speed differential at timestep t
#   [4]  al_t  =  leader acceleration at timestep t 
#
#   [5]  df_t  =  follower/ego distance between at timestep t 
#   [6]  rf_t  =  follower/ego speed differential at timestep t
#   [7]  af_t  =  follower acceleration at timestep t 
#
# Y VALUES
#   [8]   v_tprime  =  ego velocity at timestep t+1
#   [9]   a_tprime  =  ego acceleration at timestep t+1
#   [10]  xl_tprime  =  leader position at timestep t+1
#   [11]  vl_tprime  =  leader velocity at timestep t+1
#   [12]  xf_tprime  =  follower position at timestep t+1
#   [13]  vf_tprime  =  leader velocity at timestep t+1
#
"""
def compileVehicleInfo(vehicles):
    print "compiling vehicle info..."
    output = {}      # output dict, contents detailed in function header
    MAX_DIST = 200   # maximum distance from ego vehicle that we care about

    # Iterate through all vehicles
    for id in vehicles:
        currVehicleInfo = vehicles[id]

        # Iterate through all timesteps in current vehicle's info
        for t in currVehicleInfo:

            """ Ego vehicle information """
            # EGO velocity at t, acceleration at t,
            # velocity at t+1, acceleration at t+1
            ego_info = getEgoInfo(t, currVehicleInfo)


            """ Preceding/Trailing information """
            # LEADER distance from ego at t, velocity difference at t, acceleration at t,
            #        position at t+1, velocity at t+1
            # FOLLOWER distance from ego at t, velocity difference at t, acceleration at t,
            #          position at t+1, velocity at t+1
            leader_follower_info = getLeaderFollowerInfo(vehicles, t, currVehicleInfo, MAX_DIST)


            """ Neighboring vehicle information """
            # TOP LEFT distance from ego at t, velocity difference at t,
            #          distance from ego at t+1, velocity difference at t+1,
            # LEFT distance from ego at t, velocity difference at t,
            #      distance from ego at t+1, velocity difference at t+1,
            # BOT LEFT distance from ego at t, velocity difference at t,
            #          distance from ego at t+1, velocity difference at t+1,
            # TOP RIGHT distance from ego at t, velocity difference at t,
            #           distance from ego at t+1, velocity difference at t+1,
            # RIGHT distance from ego at t, velocity difference at t,
            #       distance from ego at t+1, velocity difference at t+1,
            # BOT RIGHT distance from ego at t, velocity difference at t,
            #           distance from ego at t+1, velocity difference at t+1
            # neighboring_info = getNeighboringInfo(vehicles, t, id, MAX_DIST)


            """ Add to output """
            # X VALUES
            info = ego_info[0]
            info.extend(leader_follower_info[0])
            # Y VALUES
            info.extend(ego_info[1])
            info.extend(leader_follower_info[1])

            if id in output:
                output[id].append(info)
            else:
                output[id] = []
                output[id].append(info)

    return output



"""
# Outputs a list of ego vehicle information for the given timestep
# Params: 
#   timestep, ego vehicle info
# Returns:
#   List = [velocity at t, acceleration at t, velocity at t+1, acceleration at t+1]
#
"""
def getEgoInfo(t, currVehicleInfo):
    v_t = currVehicleInfo[t][0]  # ego speed
    a_t = currVehicleInfo[t][1]  # ego acceleration
    v_tprime = 0  # ego speed, 0 means no next time step
    a_tprime = 0  # ego acceleration, 0 means no next time step

    if t + 1 in currVehicleInfo:
        v_tprime = currVehicleInfo[t + 1][0]
        a_tprime = currVehicleInfo[t + 1][1]

    curr_info = [v_t, a_t]
    next_info = [v_tprime, a_tprime]
    return [curr_info, next_info]



"""
# Outputs a list of information for the preceding and trailing vehicles for the given timestep
# Params:
#   list of vehicle info, timestep, ego vehicle info, maximum distance to be considered neighboring
# Returns:
#   List = [leader/ego distance between at t, leader/ego speed differential at t, acceleration at t,
#           leader position at t+1, leader velocity at t+1
#           follower/ego distance between at t, follower/ego distance between at t, acceleration at t,
#           follower position at t+1, follower velocity at t+1]
#
"""
def getLeaderFollowerInfo(vehicles, t, currVehicleInfo, MAX_DIST):

    # Ego vehicle
    v_t  = currVehicleInfo[t][0]        # ego velocity
    x_t  = currVehicleInfo[t][3]        # ego location

    # Leader vehicle --> 0 means no leader
    dl_t = 0        # headway between leader/ego
    rl_t = 0        # velocity difference with leader/ego
    al_t = 0        # acceleration of leader

    # Follower vehicle --> 0 means no follower
    df_t = 0        # headway between follower
    rf_t = 0        # speed difference with follower
    af_t = 0        # acceleration of follower


    """ X VALUES (current timestep) """
    # Leader vehicle current info
    leader = currVehicleInfo[t][4]
    if leader in vehicles:
        leader_info = vehicles[leader]
        if t in leader_info:
            dl_t = leader_info[t][3] - x_t
            # Check for maximum distance
            if dl_t > MAX_DIST:
                dl_t = 0
            else:
                rl_t = v_t - leader_info[t][0]
                al_t = leader_info[t][1]


    # Follower vehicle current info
    follower = currVehicleInfo[t][5]
    if follower in vehicles:
        follower_info = vehicles[follower]
        if t in follower_info:
            df_t = x_t - follower_info[t][3]
            # Check for maximum distance
            if df_t > MAX_DIST:
                df_t = 0
            else:
                rf_t = v_t - follower_info[t][0]
                af_t = follower_info[t][1]

    curr_info = [dl_t, rl_t, al_t, df_t, rf_t, af_t]


    """ Y VALUES (next timestep) """
    next_info = [0, 0, 0, 0]
    # Ego vehicle
    if t+1 in currVehicleInfo:
        x_tprime  = currVehicleInfo[t+1][3]        # ego location at t+1

        # Leader vehicle --> 0 means no leader
        xl_tprime = 0        # position of leader at t+1
        vl_tprime = 0        # velocity of leader at t+1

        # Follower vehicle --> 0 means no follower
        xf_tprime = 0        # position of follower at t+1
        vf_tprime = 0        # velocity of follower at t+1


        # Leader vehicle next timestep info
        leader = currVehicleInfo[t+1][4]
        if leader in vehicles:
            leader_info = vehicles[leader]
            if t+1 in leader_info:
                dl_tprime = leader_info[t+1][3] - x_tprime
                # Check for maximum distance
                if dl_tprime <= MAX_DIST:
                    xl_tprime = leader_info[t+1][3]
                    vf_tprime = leader_info[t+1][0]


        # Follower vehicle next timestep info
        follower = currVehicleInfo[t+1][5]
        if follower in vehicles:
            follower_info = vehicles[follower]
            if t+1 in follower_info:
                df_tprime = x_tprime - follower_info[t+1][3]
                # Check for maximum distance
                if df_tprime <= MAX_DIST:
                    xf_tprime = follower_info[t+1][3]
                    vf_tprime = follower_info[t+1][0]

        next_info = [xl_tprime, vl_tprime, xf_tprime, vf_tprime]

    return [curr_info, next_info]



"""
# Makes sure that each chunk of consecutive data corresponds to a 12 second (120 timestepsPerBatch) stretch from the same vehicle
# Params:
#   Dictionary of {Vehicle id : [list of [List of timestep info]]}
#   Output of compileVehicleInfo()
# Returns:
#   List that can easily be divided into 12 second intervals and converted to tensors for input to network
#
"""
def organizeIntoTimesteps(vehicle_info, timestepsPerBatch):

    output = []

    # Iterate through id's
    for id in vehicle_info:
        currInfo = []
        timestepCounter = 0
        # Iterate through timesteps for the given vehicle
        for t, info in enumerate(vehicle_info[id]):
            # If we've compiled data on this vehicle corresponding to a full batch, add to output
            if timestepCounter == timestepsPerBatch:
                output.extend(currInfo)
                currInfo = []
                timestepCounter = 0

            currInfo.append(info)
            timestepCounter += 1

    return output



"""
# Writes the given list of vehicle info to a csv file
"""
def outputToFile(filename, vehicle_info):
    print "writing to file..."
    fn = "../../../intermediate_data/" + filename
    with open(fn, "w") as f:
        writer = csv.writer(f)
        writer.writerows(vehicle_info)



if __name__ == "__main__":
    outputfile = "ngsim-intermediate.csv"
    print "Using file \'" + outputfile + "\' as output file."

    with open("../../../data/ngsim.csv", "r") as f:
        vehicle_info = filterByVehicleID(f)
        vehicle_info = compileVehicleInfo(vehicle_info)
        vehicle_info = organizeIntoTimesteps(vehicle_info, 120)
        outputToFile(outputfile, vehicle_info)












