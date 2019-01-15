require 'torch'

cmd = torch.CmdLine()
cmd:text()
cmd:text('Converting csv data to tensors for input to network')
cmd:text()
cmd:text('Options')
cmd:option('-csvfile', '', 'csv file to read data from')
cmd:option('-inputsfile', 'inputs.t7', '.t7 file to write input feature tensors to')
cmd:option('-labelsfile', 'labels.t7', '.t7 file to write label tensors to')
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
-- Tensor to store inputs, will end up being length x 120 x 18 x 1 dimensions -- 
local inputs = torch.zeros(1, 120, 18)
-- Tensor to store labels, will end up being length x 120 x 18 x 1 dimensions -- 
local labels = torch.zeros(1, 120, 18)


-- Read through csv input file, writing data to tensors --
print("reading from file '" .. opt.csvfile .. "'...")
local count = 0
for line in io.lines(opt.csvfile) do
    count = count + 1
    -- keep track of this line's input and label 
    local input = torch.zeros(1, 18)
    local label = torch.zeros(1, 18)

    local i = 1
    -- split line on commas
    for d in string.gmatch(line, "([^,]+)") do
        -- velocity, acc, leader dist, leader rel
        if i >= 1 and i <= 4 then
            input[{1, i}] = d
        -- follower dist/rel
        elseif i == 6 or i == 7 then
            input[{1, i-1}] = d
        -- topleft dist/rel
        elseif i == 9 or i == 10 then
            input[{1, i-2}] = d
        -- left dist/rel
        elseif i == 12 or i == 13 then
            input[{1, i-3}] = d
        -- botleft dist/rel
        elseif i == 15 or i == 16 then
            input[{1, i-4}] = d
        -- topright dist/rel
        elseif i == 18 or i == 19 then
            input[{1, i-5}] = d
        -- right dist/rel
        elseif i == 21 or i == 22 then
            input[{1, i-6}] = d
        -- botright dist/rel
        elseif i == 24 or i == 25 then
            input[{1, i-7}] = d
        -- vtprime, atprime, leader x/v, follower x/v, neighboring x/v (see wiki)
        elseif i >= 27 then
            label[{1, i-26}] = d
        end
        i = i + 1
    end


    -- increment batch index, wrap around if batch full
    batch_index = batch_index + 1
    if batch_index > 120 then
        batch_index = 1
        length_index = length_index + 1
        inputs = torch.cat(inputs, torch.zeros(1, 120, 18), 1)
        labels = torch.cat(labels, torch.zeros(1, 120, 18), 1)
    end
    -- store current tensors
    inputs[{length_index, batch_index}] = input
    labels[{length_index, batch_index}] = label

end

print("saving tensors...")
torch.save(opt.inputsfile, inputs)
torch.save(opt.labelsfile, labels)






