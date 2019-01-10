require 'torch'
require 'nn'
require 'nngraph'
require 'optim'
require '../util/normalNLL'
require 'csvigo'
convert = require '../util/convert'
local CarDataLoader = require '../util/CarDataLoader'


-- Load a network from checkpoint
function load_net(valSet)

	-- Specify directory containing stored nets
	local net_dir = '/Users/ian/development/final/nets/ten_fold_followers_basic'

	-- Specify RNN checkpoint file
	checkpoint_file = net_dir .. '/followers-basic-valSet' .. valSet .. '.00.t7'

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
	   	if opt.gpuid >=0 then h_init = h_init:cuda() end
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
-- Propagate basic state [d(t), r(t), s(t), a(t)]
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
            x[{t+1, {}, 3}] = -x[{t+1, {}, 4}] + s_lead[t - 20] -- r(t + dt)
            x[{t+1, {}, 5}] = acc -- a(t + dt)
        end   
    end
    -- return 100 timesteps of propagated state data [d(t), r(t), s(t), a(t)]
    return x[{{22, 121}, {}, {2, 5}}]
end

-- Propagate augmented state [dl(t), rl(t), s(t), a(t), df(t), rf(t)]
local function propagateFollowerBasic(states, target, x_lead, s_lead, x_follower, s_follower, loader)

	-- Create tensor to hold state at each time step, starting 2 seconds back
	-- state, x = [dl(t), rl(t), s(t), a(t), df(t), rf(t)]
	local x = torch.zeros(121, m, 7)
    local input = torch.zeros(120, m, 6)

    -- Initialize internal state
    current_state = init_state

    -- Find lead, follow vehicle position at t = 0
    local x_lead0 = x_lead[1] - 0.1*s_lead[1]
    local x_follow0 = x_follow[1] - 0.1*s_follow[1]

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
            x[{t+1, {}, 3}] = -x[{t+1, {}, 4}] + s_lead[t - 20]   -- rl(t + dt)     lead speed diff
            x[{t+1, {}, 5}] = acc                                 -- a(t + dt)      ego acc
            x[{t+1, {}, 6}] = x[{t+1, {}, 1}] + x_follow[t - 20]  -- df(t + dt)     follow headway
            x[{t+1, {}, 7}] = x[{t+1, {}, 4}] + s_follow[t - 20]  -- rf(t + dt)     follow speed diff
        end   
    end
    -- return 100 timesteps of propagated state data [dl(t), rl(t), s(t), a(t), df(t), rf(t)]
    return x[{{22, 121}, {}, {2, 7}}]
end



-- Function to write tensors to csv file in desired directory
local function toFile(dir, data, fold)
    if opt.mixture_size > 0 then
        csvigo.save(dir .. '/mixture_' .. fold .. '.csv', torch.totable(data))
    else
        csvigo.save(dir .. '/softmax_' .. fold .. '.csv', torch.totable(data))
    end
end

-- Load state, velocity, and acceleration data
loader = CarDataLoader.create(10, 10, true)

-- Loop over each fold in cross-validation
for fold = 1, 10 do
   
    -- local input_size = 4
    local input_size = 6

    -- Load inputs/targets
    loader.valSet = fold
    local states = loader.X[fold]
    local target = loader.vel[fold]
    local acc = loader.Y[fold]
    local x_lead = loader.x_lead[fold]
    local s_lead = loader.s_lead[fold]

    -- Follower inputs
    local x_follow, s_follow
    if input_size == 6 then
        x_follow = loader.x_follow[fold]
        s_follow = loader.s_follow[fold]
    end

    --Define # of trajectories to simulate
    m = 50

    -- Load network
    load_net(fold)
    
    -- Reshape data
    states = torch.reshape(states, loader.batches*opt.batch_size, 120, input_size)
    protos.rnn:evaluate()

    -- True values of target variables to be simulated
    target = torch.reshape(target, loader.batches*opt.batch_size, 120) -- velocity
    acc = torch.reshape(acc, loader.batches*opt.batch_size, 120)
    x_lead = torch.reshape(x_lead, loader.batches*opt.batch_size, 120)
    s_lead = torch.reshape(s_lead, loader.batches*opt.batch_size, 120)
    collectgarbage()

    -- Initialize tensors to hold data
	local sim = torch.zeros(10*loader.batches, 100, 50, input_size)
	local real = torch.zeros(10*loader.batches, 100, input_size)
	local size = 0

    print('Propagating trajectories in fold ' .. fold)

    -- Loop over all inputs
    for i = 1, states:size(1) do
        if target[i][100] ~= 0 then -- Avoid wasting time on abbreviated trajectories
            size = size + 1

            -- Propagate simulated trajectories and store output
            if input_size == 4 then
                sim[size] = propagateBasic(states[i], target[i], x_lead[i], s_lead[i], loader)
            elseif input_size == 6 then
                sim[size] = propagateFollowerBasic(states[i], target[i], x_lead[i], s_lead[i], x_follow[i], s_follow[i], loader)
            end

            -- Combine and store true trajectory values
            local dl = torch.cat(states[{i, {22, 120}, 1}], torch.Tensor({0}))   -- leader headway distance
            local rl = torch.cat(states[{i, {22, 120}, 2}], torch.Tensor({0}))   -- leader relative speed differential
            local ldr = torch.cat(dl, rl, 2)                                     -- store both
            local va = torch.cat(torch.cat(target[{i, {22, 120}}], torch.Tensor({0})), torch.cat(acc[{i, {22, 120}}], torch.Tensor({0})), 2)    -- velocity and acceleration
            real[size] = torch.cat(ldr, va, 2)   -- tensor size 100x4 (keep track of real trajectories for final 10 seconds)
          
            -- Add follower data to stored true trajectory values
            if input_size == 6 then
                local df = torch.cat(states[{i, {22, 120}, 5}], torch.Tensor({0}))   -- follower headway distance
                local rf = torch.cat(states[{i, {22, 120}, 6}], torch.Tensor({0}))   -- follower relative speed differential
                local fdr = torch.cat(df, rf, 2)                                     -- store both
                real[size] = torch.cat(real[size], fdr)
            end
        end
        if i%100 == 0 then print(i) end -- track progress
    end

    -- Get rid of empty values
	sim = sim[{{1, size}, {}, {}, {}}]
	real = real[{{1, size}, {}, {}}]

	-- Reshape tensors so that they can be written to csv
	sim = torch.reshape(sim, size * 100, m*input_size)   -- reshape to 2 dimensional [size*100 x m*input_size]
	real = torch.reshape(real, size * 100, input_size)   -- reshape to 2 dimensional [size*100 x input_size]
	collectgarbage()

	-- Combine tensors, rescale and shift state values
	local combine = torch.cat(sim, real, 2)

	-- Write data to csv
	toFile('/Users/ian/development/final/simulated_trajectories', combine, fold)
end

