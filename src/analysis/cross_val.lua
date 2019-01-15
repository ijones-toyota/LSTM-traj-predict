-- Modified from Jeremy Morton's implementation to handle various input sizes

require 'torch'
require 'nn'
require 'nngraph'
require 'optim'
require '../util/normalNLL'
require 'csvigo'
convert = require '../util/convert'
local CarDataLoader = require '../util/CarDataLoader'

cmd = torch.CmdLine()
cmd:text()
cmd:text('')
cmd:text()
cmd:text('Options')
-- num inputs
cmd:option('-input_size',4,'number of input features')
cmd:option('-batch_size',10,'number of sequences to train on in parallel')
cmd:option('-nfolds',10,'number of folds to use in cross-validation')
cmd:text()
-- parse input params
opt = cmd:parse(arg)


------------------------------------
-- Load a network from checkpoint --
------------------------------------
function load_net(valSet)

    -- Specify directory containing stored nets
    local net_dir, file_prefix
    if opt.input_size == 4 then
        net_dir = '/Users/ian/development/final/nets/ten_fold_ngsim_basic'
        file_prefix = '/ngsim-basic-valSet' 
    elseif opt.input_size == 6 then
        net_dir = '/Users/ian/development/final/nets/ten_fold_followers_basic'
        file_prefix = '/followers-basic-valSet' 
    elseif opt.input_size == 18 then
        net_dir = '/Users/ian/development/final/nets/ten_fold_neighbors_basic'
        file_prefix = '/neighbors-basic-valSet' 
    end
	-- Specify RNN checkpoint file
	checkpoint_file = net_dir .. file_prefix .. valSet .. '.00.t7'

	-- Load RNN from checkpoint
	print('loading an LSTM from checkpoint')
	local checkpoint = torch.load(checkpoint_file)
	protos = checkpoint.protos
	protos.rnn:evaluate()
	opt = checkpoint.opt
	bounds = checkpoint.bounds

	-- Initialize network
	init_state = {}
	for L=1,opt.num_layers do
	   	local h_init = torch.zeros(50, opt.nn_size)
	   	table.insert(init_state, h_init:clone())
	   	table.insert(init_state, h_init:clone())
	end
	state_size = #init_state
	print('Done.')
	collectgarbage()
end

-- Random sample from predicted distributions
local function sample(prediction, protos)
    if opt.mixture_size >= 1 then -- Gaussian mixture
        -- Deep neural network prediction on Gaussian mixture
        if prediction:size(2) == 2 then
            local rand_samples = torch.randn(m) -- generate m samples from standard normal distribution
            return torch.cmul(prediction[{{}, 2}], rand_samples) + prediction[{{}, 1}]
        else
            local n = prediction:size(2)/3
            local acc = torch.Tensor(m):zero()
            for i = 1, n do
                rand_samples = torch.randn(m)
                pred = torch.cmul(prediction[{{}, 2*n + i}], rand_samples) + prediction[{{}, n + i}] -- convert
                acc = acc + torch.cmul(prediction[{{}, i}], pred)
            end
            return acc
        end

    -- Softmax
    else
    	local probs = torch.exp(prediction)
        local bins = torch.multinomial(probs, 1) -- sample bins from softmax

        -- Find accelerations from predicted bins and bin boundaries
        local acc = torch.zeros(m)
        for i = 1, m do
            acc[i] = bounds[bins[i]:squeeze()] + torch.rand(1):squeeze() * (bounds[bins[i]:squeeze() + 1] - bounds[bins[i]:squeeze()])
        end
        return acc
    end
end


----------------------------------------------------------------------------------------------------------
-- PROPAGATION FUNCTIONS --
-- Functions to generate velocity predictions at 1, ..., 10 sec horizons using recurrent neural network -- 
----------------------------------------------------------------------------------------------------------
-- Propagate basic state [d(t), r(t), s(t), a(t)] --
local function propagateBasic(states, target, x_lead, s_lead, loader)

	-- Create tensor to hold state at each time step, starting 2 seconds back
	-- state, x = [d(t), r(t), s(t), a(t)]
	local x = torch.zeros(121, m, 5)
    local input = torch.zeros(120, m, 4)

    -- Initialize internal state
    current_state = init_state

    -- Find lead vehicle position at t = 0
    local x_lead0 = x_lead[1] - 0.1*s_lead[1]

    -- Loop over all simulated trajectories
    for i = 1, m do
        -- Fill in vehicle position at last time step before propagation begins
       x[{21, i, 1}] = -states[{21, 1}] + x_lead0

       -- Fill in states at t = -2 sec to t = 0
	   x[{{1, 21}, i, {2, 5}}] = states[{{1, 21}, {}}]

       -- Fill in input with true state for propagating oracle
       input[{{}, i}] = states
    end

    -- forward pass
    -- NOTE: input and output to the network differ by one time step i.e. if input corresponds to
    -- t = 21 then output will correspond to t = 22.  Mapping from t -> time: t = 21 <-> time = 0 sec
    for t=1,120 do
        local lst = protos.rnn:forward{convert.augmentInput(x[{t, {}, {2, 5}}], loader), unpack(current_state)}
        current_state = {}
        for i=1,state_size do table.insert(current_state, lst[i]) end
        if t > 20 then -- Generate predictions after 2 sec
            local prediction = lst[#lst] -- last element holds the acceleration prediction
            local acc = sample(prediction, protos)

        	x[{t+1, {}, 1}] = x[{t, {}, 1}] + torch.mul(x[{t, {}, 4}], 0.1) + torch.mul(acc, 0.01) -- x_ego(t + dt)
            x[{t+1, {}, 2}] = -x[{t+1, {}, 1}] + x_lead[t - 20] -- d(t + dt)
            x[{t+1, {}, 4}] = x[{t, {}, 4}] + torch.mul(acc, 0.1) -- s(t + dt)
            x[{t+1, {}, 3}] = x[{t+1, {}, 4}] - s_lead[t - 20] -- r(t + dt)
            x[{t+1, {}, 5}] = acc -- a(t + dt)
        end   
    end
    -- return 100 timesteps of propagated state data [d(t), r(t), s(t), a(t)]
    return x[{{22, 121}, {}, {2, 5}}]
end

-- Propagate augmented state [dl(t), rl(t), s(t), a(t), df(t), rf(t)] --
local function propagateFollowerBasic(states, target, x_lead, s_lead, x_follow, s_follow, loader)

	-- Create tensor to hold state at each time step, starting 2 seconds back
	-- state, x = [dl(t), rl(t), s(t), a(t), df(t), rf(t)]
	local x = torch.zeros(121, m, 7)
    local input = torch.zeros(120, m, 6)

    -- Initialize internal state
    current_state = init_state

    -- Find lead vehicle position at t = 0
    local x_lead0 = x_lead[1] - 0.1*s_lead[1]

    -- Loop over all simulated trajectories
    for i = 1, m do
        -- Fill in vehicle position at last time step before propagation begins
       x[{21, i, 1}] = -states[{21, 1}] + x_lead0

       -- Fill in states at t = -2 sec to t = 0
	   x[{{1, 21}, i, {2, 7}}] = states[{{1, 21}, {}}]

       -- Fill in input with true state for propagating oracle
       input[{{}, i}] = states
    end

    -- forward pass
    -- NOTE: input and output to the network differ by one time step i.e. if input corresponds to
    -- t = 21 then output will correspond to t = 22.  Mapping from t -> time: t = 21 <-> time = 0 sec
    for t=1,120 do
        local lst = protos.rnn:forward{convert.augmentInput(x[{t, {}, {2, 7}}], loader), unpack(current_state)}
        current_state = {}
        for i=1,state_size do table.insert(current_state, lst[i]) end
        if t > 20 then -- Generate predictions after 2 sec
            local prediction = lst[#lst] -- last element holds the acceleration prediction
            local acc = sample(prediction, protos)

        	x[{t+1, {}, 1}] = x[{t, {}, 1}] + torch.mul(x[{t, {}, 4}], 0.1) + torch.mul(acc, 0.01) -- x_ego(t + dt)
            x[{t+1, {}, 2}] = -x[{t+1, {}, 1}] + x_lead[t - 20]   -- dl(t + dt)     lead headway
            x[{t+1, {}, 4}] = x[{t, {}, 4}] + torch.mul(acc, 0.1) -- s(t + dt)      ego speed
            x[{t+1, {}, 3}] = x[{t+1, {}, 4}] - s_lead[t - 20]   -- rl(t + dt)     lead speed diff
            x[{t+1, {}, 5}] = acc                                 -- a(t + dt)      ego acc
            x[{t+1, {}, 6}] = x[{t+1, {}, 1}] - x_follow[t - 20]  -- df(t + dt)     follow headway
            x[{t+1, {}, 7}] = x[{t+1, {}, 4}] - s_follow[t - 20]  -- rf(t + dt)     follow speed diff
        end   
    end
    -- return 100 timesteps of propagated state data [dl(t), rl(t), s(t), a(t), df(t), rf(t)]
    return x[{{22, 121}, {}, {2, 7}}]
end

-- Propagate augmented state that includes neighboring vehicles
local function propagateNeighborsBasic(states, target, x_lead, s_lead, x_follow, s_follow, 
                                       x_1, s_1, x_2, s_2, x_3, s_3,
                                       x_4, s_4, x_5, s_5, x_6, s_6,
                                       loader)

	-- Create tensor to hold state at each time step, starting 2 seconds back
    -- state, x = [s(t), a(t), dl(t), rl(t), df(t), rf(t),
    --             d1(t), r1(t), d2(t), r2(t), d3(t), r3(t),
    --             d4(t), r4(t), d5(t), r5(t), d6(t), r6(t)]
	local x = torch.zeros(121, m, 19)
    local input = torch.zeros(120, m, 18)

    -- Initialize internal state
    current_state = init_state

    -- Find lead vehicle position at t = 0
    local x_lead0 = x_lead[1] - 0.1*s_lead[1]
 
    -- Loop over all simulated trajectories
    for i = 1, m do
        -- Fill in vehicle position at last time step before propagation begins
       x[{21, i, 1}] = -states[{21, 1}] + x_lead0

       -- Fill in states at t = -2 sec to t = 0
	   x[{{1, 21}, i, {2, 19}}] = states[{{1, 21}, {}}]

       -- Fill in input with true state for propagating oracle
       input[{{}, i}] = states
    end

    -- forward pass
    -- NOTE: input and output to the network differ by one time step i.e. if input corresponds to
    -- t = 21 then output will correspond to t = 22.  Mapping from t -> time: t = 21 <-> time = 0 sec
    for t=1,120 do
        local lst = protos.rnn:forward{convert.augmentInput(x[{t, {}, {2, 19}}], loader), unpack(current_state)}
        current_state = {}
        for i=1,state_size do table.insert(current_state, lst[i]) end
        if t > 20 then -- Generate predictions after 2 sec
            local prediction = lst[#lst] -- last element holds the acceleration prediction
            local acc = sample(prediction, protos)

            x[{t+1, {}, 1}] = x[{t, {}, 1}] + torch.mul(x[{t, {}, 4}], 0.1) + torch.mul(acc, 0.01) -- x_ego(t + dt)
            x[{t+1, {}, 2}] = x[{t, {}, 2}] + torch.mul(acc, 0.1) -- s(t + dt)      ego speed
            x[{t+1, {}, 3}] = acc                                 -- a(t + dt)      ego acc

            x[{t+1, {}, 4}] = -x[{t+1, {}, 1}] + x_lead[t - 20]   -- dl(t + dt)     lead headway
            x[{t+1, {}, 5}] = x[{t+1, {}, 2}] - s_lead[t - 20]    -- rl(t + dt)     lead speed diff
            x[{t+1, {}, 6}] = x[{t+1, {}, 1}] + x_follow[t - 20]  -- df(t + dt)     follow headway
            x[{t+1, {}, 7}] = x[{t+1, {}, 2}] - s_follow[t - 20]  -- rf(t + dt)     follow speed diff

            x[{t+1, {}, 8}] = -x[{t+1, {}, 1}] + x_1[t - 20]    -- d(t + dt)     car1 headway
            x[{t+1, {}, 9}] = x[{t+1, {}, 2}] - s_1[t - 20]     -- r(t + dt)     car1 speed diff
            x[{t+1, {}, 10}] = -x[{t+1, {}, 1}] + x_2[t - 20]   -- d(t + dt)     car2 headway
            x[{t+1, {}, 11}] = x[{t+1, {}, 2}] - s_2[t - 20]    -- r(t + dt)     car2 speed diff
            x[{t+1, {}, 12}] = x[{t+1, {}, 1}] - x_3[t - 20]    -- d(t + dt)     car3 headway
            x[{t+1, {}, 13}] = x[{t+1, {}, 2}] - s_3[t - 20]    -- r(t + dt)     car3 speed diff
            x[{t+1, {}, 14}] = -x[{t+1, {}, 1}] + x_4[t - 20]   -- d(t + dt)     car4 headway
            x[{t+1, {}, 15}] = x[{t+1, {}, 2}] - s_4[t - 20]    -- r(t + dt)     car4 speed diff
            x[{t+1, {}, 16}] = -x[{t+1, {}, 1}] + x_5[t - 20]   -- d(t + dt)     car5 headway
            x[{t+1, {}, 17}] = x[{t+1, {}, 2}] - s_5[t - 20]    -- r(t + dt)     car5 speed diff
            x[{t+1, {}, 18}] = x[{t+1, {}, 1}] - x_6[t - 20]    -- d(t + dt)     car6 headway
            x[{t+1, {}, 19}] = x[{t+1, {}, 2}] - s_6[t - 20]    -- r(t + dt)     car6 speed diff
        end   
    end
    -- return 100 timesteps of propagated state data [s(t), a(t), dl(t), rl(t), df(t), rf(t), 
    --                                                d1(t), r1(t), d2(t), r2(t), d3(t), r3(t),
    --                                                d4(t), r4(t), d5(t), r5(t), d6(t), r6(t)]
    return x[{{22, 121}, {}, {2, 19}}]
end


----------------------------------------------------------------
-- Function to write tensors to csv file in desired directory --
----------------------------------------------------------------
local function toFile(dir, data, fold)
    if opt.mixture_size > 0 then
        csvigo.save(dir .. '/mixture_' .. fold .. '.csv', torch.totable(data))
    else
        csvigo.save(dir .. '/softmax_' .. fold .. '.csv', torch.totable(data))
    end
end


-----------------------------------------------------------
-- Load data and loop over each fold in cross-validation --
-----------------------------------------------------------
loader = CarDataLoader.create(opt.nfolds, opt.batch_size, opt.input_size, true)

for fold = 1, 10 do

    -- Load inputs/targets
    loader.valSet = fold
    local states = loader.X[fold]
    local target = loader.vel[fold]
    local acc = loader.Y[fold]
    local x_lead = loader.x_lead[fold]
    local s_lead = loader.s_lead[fold]

    -- Add follower inputs
    local x_follow, s_follow
    if opt.input_size == 6 or opt.input_size == 18 then
        x_follow = loader.x_follow[fold]
        s_follow = loader.s_follow[fold]
    end
   
    local x_1, s_1, x_2, s_2, x_3, s_3, x_4, s_4, x_5, s_5, x_6, s_6
    -- Add neighbors inputs
    if opt.input_size == 18 then
        x_1 = loader.x_1[fold]
        s_1 = loader.s_1[fold]
        x_2 = loader.x_2[fold]
        s_2 = loader.s_2[fold]
        x_3 = loader.x_3[fold]
        s_3 = loader.s_3[fold]
        x_4 = loader.x_4[fold]
        s_4 = loader.s_4[fold]
        x_5 = loader.x_5[fold]
        s_5 = loader.s_5[fold]
        x_6 = loader.x_6[fold]
        s_6 = loader.s_6[fold]
    end


    --Define # of trajectories to simulate
    m = 50

    local input_size = opt.input_size   -- shouldn't be necessary if nets are trained with this param
    -- Load network
    load_net(fold)
    opt.input_size = input_size

    -- Reshape data
    states = torch.reshape(states, loader.batches*opt.batch_size, 120, opt.input_size)
    protos.rnn:evaluate()

    -- True values of target variables to be simulated
    target = torch.reshape(target, loader.batches*opt.batch_size, 120) -- velocity
    acc = torch.reshape(acc, loader.batches*opt.batch_size, 120)
    x_lead = torch.reshape(x_lead, loader.batches*opt.batch_size, 120)
    s_lead = torch.reshape(s_lead, loader.batches*opt.batch_size, 120)

    -- Reshape follower data
    if opt.input_size == 6 or opt.input_size == 18 then
        x_follow = torch.reshape(x_follow, loader.batches*opt.batch_size, 120)
        s_follow = torch.reshape(s_follow, loader.batches*opt.batch_size, 120)
    end
    -- Reshape neighbors data
    if opt.input_size == 18 then
        x_1 = torch.reshape(x_1, loader.batches*opt.batch_size, 120)
        s_1 = torch.reshape(s_1, loader.batches*opt.batch_size, 120)
        x_2 = torch.reshape(x_2, loader.batches*opt.batch_size, 120)
        s_2 = torch.reshape(s_2, loader.batches*opt.batch_size, 120)
        x_3 = torch.reshape(x_3, loader.batches*opt.batch_size, 120)
        s_3 = torch.reshape(s_3, loader.batches*opt.batch_size, 120)
        x_4 = torch.reshape(x_4, loader.batches*opt.batch_size, 120)
        s_4 = torch.reshape(s_4, loader.batches*opt.batch_size, 120)
        x_5 = torch.reshape(x_5, loader.batches*opt.batch_size, 120)
        s_5 = torch.reshape(s_5, loader.batches*opt.batch_size, 120)
        x_6 = torch.reshape(x_6, loader.batches*opt.batch_size, 120)
        s_6 = torch.reshape(s_6, loader.batches*opt.batch_size, 120)
    end
    collectgarbage()

    -- Initialize tensors to hold data
	local sim = torch.zeros(10*loader.batches, 100, 50, opt.input_size)
	local real = torch.zeros(10*loader.batches, 100, opt.input_size)
	local size = 0

    print('Propagating trajectories in fold ' .. fold)
    print("Number of inputs to iterate through = " .. states:size(1))

    -- Loop over all inputs
    for i = 1, states:size(1) do
        if target[i][100] ~= 0 then -- Avoid wasting time on abbreviated trajectories
            size = size + 1

            -- Propagate simulated trajectories and store output
            if opt.input_size == 4 then
                sim[size] = propagateBasic(states[i], target[i], x_lead[i], s_lead[i], loader)
            elseif opt.input_size == 6 then
                sim[size] = propagateFollowerBasic(states[i], target[i], x_lead[i], s_lead[i], x_follow[i], s_follow[i], loader)
            elseif opt.input_size == 18 then
                sim[size] = propagateNeighborsBasic(states[i], target[i], x_lead[i], s_lead[i], x_follow[i], s_follow[i], 
                                                    x_1[i], s_1[i], x_2[i], s_2[i], x_3[i], s_3[i], 
                                                    x_4[i], s_4[i], x_5[i], s_5[i], x_6[i], s_6[i], 
                                                    loader)
            end

            -- Combine and store true trajectory values
            local dl = torch.cat(states[{i, {22, 120}, 1}], torch.Tensor({0}))   -- leader headway distance
            local rl = torch.cat(states[{i, {22, 120}, 2}], torch.Tensor({0}))   -- leader relative speed differential
            local ldr = torch.cat(dl, rl, 2)                                     -- store both
            local va = torch.cat(torch.cat(target[{i, {22, 120}}], torch.Tensor({0})), torch.cat(acc[{i, {22, 120}}], torch.Tensor({0})), 2)    -- velocity and acceleration
            
            if opt.input_size == 4 then
                real[size] = torch.cat(ldr, va, 2)   -- tensor size 100x4 (keep track of real trajectories for final 10 seconds)
            -- Add follower data to stored true trajectory values
            elseif opt.input_size == 6 then
                local ldrva = torch.cat(ldr, va, 2)
                local df = torch.cat(states[{i, {22, 120}, 5}], torch.Tensor({0}))   -- follower headway distance
                local rf = torch.cat(states[{i, {22, 120}, 6}], torch.Tensor({0}))   -- follower relative speed differential
                local fdr = torch.cat(df, rf, 2)                                     -- store both
                real[size] = torch.cat(ldrva, fdr, 2)
            elseif opt.input_size == 18 then
                -- ego vel/acc
                local va = torch.cat(torch.cat(target[{i, {22, 120}}], torch.Tensor({0})), torch.cat(acc[{i, {22, 120}}], torch.Tensor({0})), 2)    -- velocity and acceleration
                -- leader distance headway/rel speed diff 
                local drl = torch.cat(torch.cat(states[{i, {22, 120}, 3}], torch.Tensor({0})), torch.cat(states[{i, {22, 120}, 4}], torch.Tensor({0})), 2)  -- store both
                local drf = torch.cat(torch.cat(states[{i, {22, 120}, 5}], torch.Tensor({0})), torch.cat(states[{i, {22, 120}, 6}], torch.Tensor({0})), 2)  -- store both

                -- neighbors distance headway and relative speed differential
                local dr1 = torch.cat(torch.cat(states[{i, {22, 120}, 7}], torch.Tensor({0})), torch.cat(states[{i, {22, 120}, 8}], torch.Tensor({0})), 2)      -- store both
                local dr2 = torch.cat(torch.cat(states[{i, {22, 120}, 9}], torch.Tensor({0})), torch.cat(states[{i, {22, 120}, 10}], torch.Tensor({0})), 2)     -- store both
                local dr3 = torch.cat(torch.cat(states[{i, {22, 120}, 11}], torch.Tensor({0})), torch.cat(states[{i, {22, 120}, 12}], torch.Tensor({0})), 2)    -- store both
                local dr4 = torch.cat(torch.cat(states[{i, {22, 120}, 13}], torch.Tensor({0})), torch.cat(states[{i, {22, 120}, 14}], torch.Tensor({0})), 2)    -- store both
                local dr5 = torch.cat(torch.cat(states[{i, {22, 120}, 15}], torch.Tensor({0})), torch.cat(states[{i, {22, 120}, 16}], torch.Tensor({0})), 2)    -- store both
                local dr6 = torch.cat(torch.cat(states[{i, {22, 120}, 17}], torch.Tensor({0})), torch.cat(states[{i, {22, 120}, 18}], torch.Tensor({0})), 2)    -- store both

                -- combine tensors
                local real_neighbors = torch.cat(va, drl, 2)
                real_neighbors = torch.cat(real_neighbors, drf, 2)
                real_neighbors = torch.cat(real_neighbors, dr1, 2)
                real_neighbors = torch.cat(real_neighbors, dr2, 2)
                real_neighbors = torch.cat(real_neighbors, dr3, 2)
                real_neighbors = torch.cat(real_neighbors, dr4, 2)
                real_neighbors = torch.cat(real_neighbors, dr5, 2)
                real_neighbors = torch.cat(real_neighbors, dr6, 2)

                real[size] = real_neighbors
            end
        end
        if i%100 == 0 then print(i) end -- track progress
        if i == 300 then break end
    end

    -- Get rid of empty values
	sim = sim[{{1, size}, {}, {}, {}}]
	real = real[{{1, size}, {}, {}}]

	-- Reshape tensors so that they can be written to csv
	sim = torch.reshape(sim, size * 100, m*opt.input_size)   -- reshape to 2 dimensional [size*100 x m*input_size]
	real = torch.reshape(real, size * 100, opt.input_size)   -- reshape to 2 dimensional [size*100 x input_size]
	collectgarbage()

	-- Combine tensors, rescale and shift state values
	local combine = torch.cat(sim, real, 2)

	-- Write data to csv
	toFile('/Users/ian/development/final/simulated_trajectories', combine, fold)
end

