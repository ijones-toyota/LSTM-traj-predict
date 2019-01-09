# LSTM-traj-predict
LSTM to predict vehicle trajectories

Sample command: th RNNtraj-predict.lua -mixture_size 2 -nn_size 128 -learning_rate 4e-3 -epochs 10 -dropout 0.25 -savenet -valSet 1

* _util_ folder contains scripts that are used for pre- and post- processing of the data
* _analysis_ folder contains scripts for computing error and other analysis metrics
* _model_ folder contains the models, adapted from Andrej Karpathy's char-rnn and Jeremy Morton's _LSTM-acc-predict_
