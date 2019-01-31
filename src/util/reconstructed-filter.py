import sys
import csv



"""
# Filter through the dataset, organizing by vehicle id and timestep (frame)
# Params:
#   reconstructed NGSIM dataset
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
        line = line.split('\t')
        id        = int(line[0].strip())        # Current vehicle ID
        frame     = int(line[1].strip())        # Current frame
        lane_id   = int(line[2].strip())        # Lane
        local_y   = float(line[3].strip())      # Y position
        v_vel     = float(line[4].strip())      # Ego velocity
        v_acc     = float(line[5].strip())      # Ego acceleration
        follower  = int(line[8].strip())        # ID of vehicle behind ego
        leader    = int(line[9].strip())        # ID of vehicle ahead of ego
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



def analyzeAcceleration(vehicle_info):

    numZeroAcc= 0
    numDataPoints = 0
    numRepeatedZeros = 0

    currZeroChain = 0
    zeroChains = {}

    for id in vehicle_info.keys():
        timesteps = vehicle_info[id]
        for t in timesteps.keys():
            numDataPoints += 1
            info = timesteps[t]

            # Acceleration value is 0, keep track
            if abs(info[1]) < 0.005:
                if currZeroChain >= 1:
                    currZeroChain += 1
                else:
                    currZeroChain = 1
                numZeroAcc += 1
            # Reset count on the number of consecutive timesteps with acceleration 0
            else:
                if currZeroChain >= 1:
                    numRepeatedZeros += 1
                    if currZeroChain in zeroChains:
                        zeroChains[currZeroChain] += 1
                    else:
                        zeroChains[currZeroChain] = 1
                currZeroChain = 0

    print("Total timesteps with acceleration = 0: " + str(numZeroAcc))
    print("Total timesteps: " + str(numDataPoints))
    print("Total timesteps with repeated 0's = " + str(numRepeatedZeros))

    for num in zeroChains:
        print(str(num) + " --> " + str(zeroChains[num]))



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
            #          position at t+1, velocity at t+1,
            # LEFT distance from ego at t, velocity difference at t,
            #      position at t+1, velocity at t+1,
            # BOT LEFT distance from ego at t, velocity difference at t,
            #          position at t+1, velocity at t+1,
            # TOP RIGHT distance from ego at t, velocity difference at t,
            #           position at t+1, velocity at t+1,
            # RIGHT distance from ego at t, velocity difference at t,
            #       position at t+1, velocity at t+1,
            # BOT RIGHT distance from ego at t, velocity difference at t,
            #           position at t+1, velocity at t+1
            neighboring_info = getNeighboringInfo(vehicles, t, id, MAX_DIST)


            """ Add to output """
            # X VALUES
            info = ego_info[0]
            info.extend(leader_follower_info[0])
            info.extend(neighboring_info[0])
            # Y VALUES
            info.extend(ego_info[1])
            info.extend(leader_follower_info[1])
            info.extend(neighboring_info[1])

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
#   list of leader and follower current and next timestep info
#   List[0] = [leader/ego distance between at t, leader/ego speed differential at t, leader acceleration at t,
#              follower/ego distance between at t, follower/ego speed differential at t, follower acceleration at t]
#   List[1] = [leader position at t+1, leader velocity at t+1, follower position at t+1, follower velocity at t+1]
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
# Outputs a list of information for the surrounding vehicles for the given timestep
# Params:
#   list of vehicle info, timestep, ego vehicle id, maximum distance to be considered neighboring
# Returns:
#   list of neighboring current and next timestep info
#   List[0] = [topleft/ego distance between at t, topleft/ego speed differential at t, topleft acceleration at t,
#              left/ego distance between at t, left/ego speed differential at t, left acceleration at t
#              bottomleft/ego distance between at t, bottomleft/ego speed differential at t, bottomleft acceleration at t
#              topright/ego distance between at t, topright/ego speed differential at t, topright acceleration at t
#              right/ego distance between at t, right/ego speed differential at t, right acceleration at t,
#              bottomright/ego distance between at t, bottomright/ego speed differential at t, bottomright acceleration at t]
#   List[1] = [topleft position at t+1, topleft velocity at t+1, 
#              left position at t+1, left velocity at t+1,
#              bottomleft position at t+1, bottomleft velocity at t+1, 
#              topright position at t+1, topright velocity at t+1,
#              right position at t+1, right velocity at t+1, 
#              bottomright position at t+1, bottomright velocity at t+1]
#
"""
def getNeighboringInfo(vehicles, t, ego_ID, MAX_DIST):
    currVehicleInfo = vehicles[ego_ID]
    leader = currVehicleInfo[t][4]
    follower = currVehicleInfo[t][5]

    """ X VALUES (current timestep) """
    ## Iterate through other vehicles, determine 3 closest vehicles in left/right lanes ##
    left_vehicles = []
    right_vehicles = []
    for otherID in vehicles:
        # skip ego vehicle and preceding/trailing cars
        if otherID == ego_ID or otherID == leader or otherID == follower:
            continue

        otherInfo = vehicles[otherID]
        # Skip if target vehicle doesn't have info for this timestep
        if t not in otherInfo:
            continue

        ego_lane = currVehicleInfo[t][2]
        ego_y = currVehicleInfo[t][3]
        other_lane = otherInfo[t][2]
        other_y = otherInfo[t][3]

        # check left lane
        if other_lane == ego_lane - 1:
            left_vehicles = checkForProximity(left_vehicles, ego_y, otherID, other_y, MAX_DIST)

        # check right lane
        elif other_lane == ego_lane + 1:
            right_vehicles = checkForProximity(right_vehicles, ego_y, otherID, other_y, MAX_DIST)

    # [distance from ego, speed differential with ego, acceleration]
    vehicle_tl_info = [0, 0, 0]  # top left
    vehicle_l_info  = [0, 0, 0]  # left
    vehicle_bl_info = [0, 0, 0]  # bot left
    vehicle_tr_info = [0, 0, 0]  # top right
    vehicle_r_info  = [0, 0, 0]  # right
    vehicle_br_info = [0, 0, 0]  # bot right

    ego_v = currVehicleInfo[t][0]
    ego_y = currVehicleInfo[t][3]
    for i, left in enumerate(left_vehicles):
        if i == 0:
            other_info = vehicles[left[0]][t]
            vehicle_tl_info = [ego_y - other_info[3], ego_v - other_info[0], other_info[1]]

        elif i == 1:
            other_info = vehicles[left[0]][t]
            vehicle_l_info = [ego_y - other_info[3], ego_v - other_info[0], other_info[1]]

        elif i == 2:
            other_info = vehicles[left[0]][t]
            vehicle_bl_info = [ego_y - other_info[3], ego_v - other_info[0], other_info[1]]

    for i, right in enumerate(right_vehicles):
        if i == 0:
            other_info = vehicles[right[0]][t]
            vehicle_tr_info = [ego_y - other_info[3], ego_v - other_info[0], other_info[1]]

        elif i == 1:
            other_info = vehicles[right[0]][t]
            vehicle_r_info = [ego_y - other_info[3], ego_v - other_info[0], other_info[1]]

        elif i == 2:
            other_info = vehicles[right[0]][t]
            vehicle_br_info = [ego_y - other_info[3], ego_v - other_info[0], other_info[1]]

    # Compile neighboring info into a single list and return it
    curr_info = vehicle_tl_info
    curr_info.extend(vehicle_l_info)
    curr_info.extend(vehicle_bl_info)
    curr_info.extend(vehicle_tr_info)
    curr_info.extend(vehicle_r_info)
    curr_info.extend(vehicle_br_info)



    """ Y VALUES (next timestep) """
    next_info = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
    if t + 1 in currVehicleInfo:
        leader = currVehicleInfo[t+1][4]
        follower = currVehicleInfo[t+1][5]

        # [position, velocity at t+1]
        vehicle_tl_info = [0, 0]  # top left
        vehicle_l_info  = [0, 0]  # left
        vehicle_bl_info = [0, 0]  # bot left
        vehicle_tr_info = [0, 0]  # top right
        vehicle_r_info  = [0, 0]  # right
        vehicle_br_info = [0, 0]  # bot right

        ## Iterate through other vehicles, determine 3 closest vehicles in left/right lanes ##
        left_vehicles = []
        right_vehicles = []
        for otherID in vehicles:
            # skip ego vehicle and preceding/trailing cars
            if otherID == ego_ID or otherID == leader or otherID == follower:
                continue

            otherInfo = vehicles[otherID]
            # Skip if target vehicle doesn't have info for this timestep
            if t+1 not in otherInfo:
                continue

            ego_lane = currVehicleInfo[t+1][2]
            ego_y = currVehicleInfo[t+1][3]
            other_lane = otherInfo[t+1][2]
            other_y = otherInfo[t+1][3]

            # check left lane
            if other_lane == ego_lane - 1:
                left_vehicles = checkForProximity(left_vehicles, ego_y, otherID, other_y, MAX_DIST)

            # check right lane
            elif other_lane == ego_lane + 1:
                right_vehicles = checkForProximity(right_vehicles, ego_y, otherID, other_y, MAX_DIST)


        for i, left in enumerate(left_vehicles):
            if i == 0:
                other_info = vehicles[left[0]][t+1]
                vehicle_tl_info = [other_info[3], other_info[0]]

            elif i == 1:
                other_info = vehicles[left[0]][t+1]
                vehicle_l_info = [other_info[3], other_info[0]]

            elif i == 2:
                other_info = vehicles[left[0]][t+1]
                vehicle_bl_info = [other_info[3], other_info[0]]

        for i, right in enumerate(right_vehicles):
            if i == 0:
                other_info = vehicles[right[0]][t+1]
                vehicle_tr_info = [other_info[3], other_info[0]]

            elif i == 1:
                other_info = vehicles[right[0]][t+1]
                vehicle_r_info = [other_info[3], other_info[0]]

            elif i == 2:
                other_info = vehicles[right[0]][t+1]
                vehicle_br_info = [other_info[3], other_info[0]]

        # Compile neighboring info into a single list and return it
        next_info = vehicle_tl_info
        next_info.extend(vehicle_l_info)
        next_info.extend(vehicle_bl_info)
        next_info.extend(vehicle_tr_info)
        next_info.extend(vehicle_r_info)
        next_info.extend(vehicle_br_info)


    return [curr_info, next_info]



"""
# Add the given [otherID, other_y] to the list if it is close enough to the ego vehicle (must be less than MAX_DIST ft away)
# Helps keep track of the 3 closest vehicles to the ego vehicle
# Params:
#   vehicle_list = left_vehicles or right_vehicles list of maxlen 3, keeps track of 3 closest vehicles to ego
#   ego_y        = ego vehicle's y position
#   otherID      = other vehicle's ID
#   other_y      = other vehicle's y position
#
# Returns:
#   Updated list of maxlen 3, ordered from smallest to largest distance to the ego vehicle
#
"""
def checkForProximity(vehicle_list, ego_y, otherID, other_y, MAX_DIST):

    if len(vehicle_list) == 0 and abs(ego_y - other_y) <= MAX_DIST:
        vehicle_list.append([otherID, other_y])
        return vehicle_list

    for i, vehicle in enumerate(vehicle_list):
        dist = abs(ego_y - vehicle[1])
        other_dist = abs(ego_y - other_y)
        if other_dist < dist:
            vehicle_list.insert(i, [otherID, other_y])
            break

    if len(vehicle_list) > 3:
        vehicle_list.pop()

    return vehicle_list



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


def analyzeZeroAccTrajectories(vehicle_info):

    timestepCounter = 0

    totalTraj = 0
    foundZeroTraj = False
    numZeroAccTraj = 0
    currZeroChain = 0
    zeroChains = {}

    TOTAL_STEPS = 50
    recorded = False
    numOneSec = 0

    # Iterate through all trajectories
    for t, info in enumerate(vehicle_info):

        # Next trajectory
        if timestepCounter == 120:
            timestepCounter = 0
            totalTraj += 1
            foundZeroTraj = False

            # Keep track of consecutive 0 acc states
            if currZeroChain in zeroChains:
                zeroChains[currZeroChain] += 1
            elif currZeroChain != 0:
                zeroChains[currZeroChain] = 1

            if currZeroChain >= TOTAL_STEPS:
                if not recorded:
                    numOneSec += 1

            recorded = False
            currZeroChain = 0


        # Within 10-second trajectory
        if timestepCounter >= 20:

            # 0 acc state
            if abs(info[1]) < 0.005:
                # Keep track of number of trajectories that contain a 0 value acc
                if foundZeroTraj == False:
                    numZeroAccTraj += 1
                    foundZeroTraj = True

                # Keep track of length of consecutive 0 acc states
                if currZeroChain >= 1:
                    currZeroChain += 1
                else:
                    currZeroChain = 1

            # non-zero acc state
            else:
                # Keep track of consecutive 0 acc states
                if currZeroChain in zeroChains:
                    zeroChains[currZeroChain] += 1
                elif currZeroChain != 0:
                    zeroChains[currZeroChain] = 1

                if currZeroChain >= TOTAL_STEPS:
                    if not recorded:
                        numOneSec += 1
                        recorded = True

                currZeroChain = 0

        timestepCounter += 1

    if timestepCounter == 120:
        totalTraj += 1

    print("Total trajectories that contain a state with 0 acc value: " + str(numZeroAccTraj))
    print("Total trajectories: " + str(totalTraj))
    print("Total trajectories that contain 1 second or more of 0 acc states: " + str(numOneSec))

    for num in zeroChains:
        print(str(num) + " --> " + str(zeroChains[num]))



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
    outputfile = "reconstructed-neighboring-intermediate.csv"
    print "Using file \'" + outputfile + "\' as output file."

    with open("../../../data/reconstructed_ngsim.tsv", "r") as f:
        vehicle_info = filterByVehicleID(f)
        # analyzeAcceleration(vehicle_info)

        vehicle_info = compileVehicleInfo(vehicle_info)
        vehicle_info = organizeIntoTimesteps(vehicle_info, 120)
        analyzeZeroAccTrajectories(vehicle_info)
        # outputToFile(outputfile, vehicle_info)












