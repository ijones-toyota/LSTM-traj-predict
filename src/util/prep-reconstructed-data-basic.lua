require 'torch'

cmd = torch.CmdLine()
cmd:text()
cmd:text('Converting csv data to tensors for input to network')
cmd:text()
cmd:text('Options')
cmd:option('-csvfile', '', 'csv file to read data from')
cmd:option('-inputsfile', 'inputs.t7', '.t7 file to write input feature tensors to')
cmd:option('-labelsfile', 'labels.t7', '.t7 file to write label tensors to')
cmd:option('-follower', false, 'whether to include follower vehicle features as well as leader vehicle')
cmd:option('-acceleration', false, 'whether to include accelerations in neighboring vehicle features')
cmd:text()

opt = cmd:parse(arg)


-- Error checking --
assert(path.exists(opt.csvfile), 'csv file not found')
if path.exists(opt.inputsfile) then
    io.write('specified inputs file already exists, overwrite (y/n)? ')
    io.flush()
    answer=io.read()
    if answer ~= 'y' then 
        os.exit()
    end
end
if path.exists(opt.labelsfile) then
    io.write('specified labels file already exists, overwrite (y/n)? ')
    io.flush()
    answer=io.read()
    if answer ~= 'y' then 
        os.exit()
    end
end    


-- Build up dimensions --
local length_index = 1   -- keep track of what outer dimension we're in
local batch_index  = 0   -- keep track of batch dimension
local num_input_features = 4    
local num_label_features = 4
if opt.follower then
    num_input_features = 6
    num_label_features = 6
end
if opt.acceleration then
    num_input_features = 8
    num_label_features = 6
end

-- Tensor to store inputs, will end up being length x 120 x num_input_features x 1 dimensions -- 
local inputs = torch.zeros(1, 120, num_input_features)
-- Tensor to store labels, will end up being length x 120 x num_label_features x 1 dimensions -- 
local labels = torch.zeros(1, 120, num_label_features)


-- Read through csv input file, writing data to tensors --
print("reading from file '" .. opt.csvfile .. "'...")
local count = 0
for line in io.lines(opt.csvfile) do
    count = count + 1
    -- keep track of this line's input and label 
    local input = torch.zeros(1, 4)
    local label = torch.zeros(1, 4)
    if opt.follower then
        input = torch.zeros(1, 6)
        label = torch.zeros(1, 6)
    end
    if opt.acceleration then
        input = torch.zeros(1, 8)
        label = torch.zeros(1, 6)
    end

    local i = 1
    -- split line on commas
    for d in string.gmatch(line, "([^,]+)") do
        -- velocity
        if i == 1 then
            input[{1, 3}] = d
        -- acceleration
        elseif i == 2 then
            input[{1, 4}] = d
        -- distance
        elseif i == 3 then
            input[{1, 1}] = d
        -- rel vel diff
        elseif i == 4 then
            input[{1, 2}] = d
        -- target vel 
        elseif i == 9 then
            label[{1, 1}] = d
        -- target acc 
        elseif i == 10 then
            label[{1, 2}] = d
        -- target leader position 
        elseif i == 11 then
            label[{1, 4}] = d
        -- target leader vel 
        elseif i == 12 then
            label[{1, 3}] = d
        end

        -- store follower distance and rel vel
        if opt.follower then
            -- follower headway distance
            if i == 6 then
                input[{1, 5}] = d
            -- follower rel vel diff
            elseif i == 7 then
                input[{1, 6}] = d
            -- target follower position
            elseif i == 13 then
                label[{1, 6}] = d
            -- target follower velocity
            elseif i == 14 then
                label[{1, 5}] = d 
            end
        end

        -- store leader/follower acceleration as well
        if opt.acceleration then
            -- leader/follower acceleration included
            if i >= 5 and i <= 8 then
                input[{1, i}] = d 
            -- target follower position 
            elseif i == 13 then
                label[{1, 6}] = d
            -- target follower velocity
            elseif i == 14 then
                label[{1, 5}] = d 
            end
        end

        i = i + 1
    end


    -- increment batch index, wrap around if batch full
    batch_index = batch_index + 1
    if batch_index > 120 then
        batch_index = 1
        length_index = length_index + 1

        inputs = torch.cat(inputs, torch.zeros(1, 120, num_input_features), 1)
        labels = torch.cat(labels, torch.zeros(1, 120, num_label_features), 1)
    end
    -- store current tensors
    inputs[{length_index, batch_index}] = input
    labels[{length_index, batch_index}] = label

end

print("saving tensors...")
torch.save(opt.inputsfile, inputs)
torch.save(opt.labelsfile, labels)






