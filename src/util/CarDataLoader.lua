-- Modified from Karpathy's char-rnn to read in NGSIM highway data

local CarDataLoader = {}
CarDataLoader.__index = CarDataLoader


--------------------------
-- Creates the loader -- 
--------------------------
function CarDataLoader.create(nfolds, batch_size, input_size)

    local self = {}
    setmetatable(self, CarDataLoader)

    -- Specify path to preprocessed data
    local x_file, y_file
    -- Basic inputs
    if input_size == 4 then
        x_file = '/Users/ian/development/final/network_data/ngsim_basic/ngsim-inputs-basic.t7'
        y_file = '/Users/ian/development/final/network_data/ngsim_basic/ngsim-labels-basic.t7'
    -- Follower basic inputs 
    elseif input_size == 6 then
        x_file = '/Users/ian/development/final/network_data/followers/inputs-followers-basic.t7'
        y_file = '/Users/ian/development/final/network_data/followers/labels-followers-basic.t7'
    end 
    
    assert(path.exists(x_file), 'Input data file not found')
    assert(path.exists(y_file), 'Target data file not found')

    -- Load data
    print('loading data files...')
    local X = torch.load(x_file)
    local Y = torch.load(y_file)
    
    -- truncate data so that it can be divided evenly into batches
    if X:size(1) % (nfolds*batch_size) ~= 0 then
        X = X[{{1, math.floor(X:size(1)/(nfolds*batch_size)) * (nfolds*batch_size)}, {}, {}}]
        Y = Y[{{1, math.floor(X:size(1)/(nfolds*batch_size)) * (nfolds*batch_size)}, {}, {}}]
    end
    
    -- Separate target velocities and accelerations (acceleration is used in training, rest is used in validation)
    local vel = Y[{{}, {}, {1}}]        -- ego vel
    local x_lead = Y[{{}, {}, {4}}]     -- lead position
    local s_lead = Y[{{}, {}, {3}}]     -- lead speed

    -- Add follower data
    local x_follow, s_follow
    if input_size == 6 then
        x_follow = Y[{{}, {}, {6}}]   -- follow position
        s_follow =  Y[{{}, {}, {5}}]  -- follow speed
    end

    Y = Y[{{}, {}, {2}}]                -- ego acc
    collectgarbage()

    -- Reshape data and store
    -- First dimension contains different folds, second dimension contains different batches
    -- Third dimension contains individual sets, fourth dimension contains 100 inputs/outputs
    -- (corresponding to 10-sec stretches), fifth dimension contains individual inputs/outputs
    self.X = torch.reshape(X, torch.LongStorage{nfolds, X:size(1)/(nfolds*batch_size), batch_size, X:size(2), X:size(3)})
    self.Y = torch.reshape(Y, torch.LongStorage{nfolds, Y:size(1)/(nfolds*batch_size), batch_size, Y:size(2)})
    self.vel = torch.reshape(vel, torch.LongStorage{nfolds, vel:size(1)/(nfolds*batch_size), batch_size, vel:size(2), vel:size(3)})
    self.x_lead = torch.reshape(x_lead, torch.LongStorage{nfolds, x_lead:size(1)/(nfolds*batch_size), batch_size, x_lead:size(2)})
    self.s_lead = torch.reshape(s_lead, torch.LongStorage{nfolds, s_lead:size(1)/(nfolds*batch_size), batch_size, s_lead:size(2)})

    -- Add follower data
    if input_size == 6 then
        self.x_follow = torch.reshape(x_follow, torch.LongStorage{nfolds, x_follow:size(1)/(nfolds*batch_size), batch_size, x_follow:size(2)})
        self.s_follow = torch.reshape(s_follow, torch.LongStorage{nfolds, s_follow:size(1)/(nfolds*batch_size), batch_size, s_follow:size(2)})
    end

    -- Calculate amount to shift and scale data by in order to have all data zero-mean and normalized
    -- Only basic data
    if input_size == 4 then
        self.shift = torch.Tensor({torch.mean(self.X[{{}, {}, {}, {}, 1}]), 
            torch.mean(self.X[{{}, {}, {}, {}, 2}]),
            torch.mean(self.X[{{}, {}, {}, {}, 3}]),
            torch.mean(self.X[{{}, {}, {}, {}, 4}])})

        self.scale = torch.Tensor({torch.std(self.X[{{}, {}, {}, {}, 1}]), 
            torch.std(self.X[{{}, {}, {}, {}, 2}]),
            torch.std(self.X[{{}, {}, {}, {}, 3}]),
            torch.std(self.X[{{}, {}, {}, {}, 4}])})
    -- Add follower data 
    elseif input_size == 6 then
        self.shift = torch.Tensor({torch.mean(self.X[{{}, {}, {}, {}, 1}]), 
            torch.mean(self.X[{{}, {}, {}, {}, 2}]),
            torch.mean(self.X[{{}, {}, {}, {}, 3}]),
            torch.mean(self.X[{{}, {}, {}, {}, 4}]),
            torch.mean(self.X[{{}, {}, {}, {}, 5}]),
            torch.mean(self.X[{{}, {}, {}, {}, 6}])})
        self.scale = torch.Tensor({torch.std(self.X[{{}, {}, {}, {}, 1}]), 
            torch.std(self.X[{{}, {}, {}, {}, 2}]),
            torch.std(self.X[{{}, {}, {}, {}, 3}]),
            torch.std(self.X[{{}, {}, {}, {}, 4}]),
            torch.std(self.X[{{}, {}, {}, {}, 5}]),
            torch.std(self.X[{{}, {}, {}, {}, 6}])})
    end 
                    
    -- Set counter to track which set is being held as validation set
    self.valSet = 0
    self.val = false

    -- Store # of folds, batch size and # batches/fold
    self.nfolds = nfolds
    self.batch_size = batch_size
    self.batches = self.X:size(2)

    -- Set index to store current batch; first value holds current fold, second value
    -- holds batch in that fold
    self.batch_ix = {1, 0}

    -- Define boolean to indicate whether there are any training batches left
    self.moreBatches = true

    print('Data load done.')
    collectgarbage()
    return self
end



---------------------------------
-- Function to load next batch --
---------------------------------
function CarDataLoader:next_batch()

    local ix = self.batch_ix

    if self.val then 
        -- Check if fold was initialized to be validation set
        if ix[1] ~= self.valSet then
            error('validation set not selected')
        end

        -- Increment to next batch, move to next fold if necessary
        if ix[2] + 1 <= self.batches then
            ix[2] = ix[2] + 1
        else
            self.moreBatches = false
        end

        -- Store new indices
        self.batch_ix = ix

    else
        -- Check if fold was initialized to be validation set
        if ix[1] == self.valSet then
            ix[1] = ix[1] + 1
        end

        -- Increment to next batch, move to next fold if necessary
        if ix[2] + 1 <= self.batches then
            ix[2] = ix[2] + 1
        else
            if ix[1] + 1 == self.valSet then
                ix = {ix[1] + 2, 1}
            else
                ix = {ix[1] + 1, 1}
            end
        end

        -- Check if any training batches are left
        if (ix[1] == self.nfolds or (ix[1] == self.nfolds - 1 and self.valSet == self.nfolds)) and
            ix[2] == self.batches then
            self.moreBatches = false
        end

        -- Store new indices
        self.batch_ix = ix
    end

    return self.X[ix[1]][ix[2]], self.Y[ix[1]][ix[2]]
end


return CarDataLoader

