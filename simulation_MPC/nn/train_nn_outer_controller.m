function train_nn_outer_controller()
% Train a neural network to approximate the outer-loop MPC:
%          [e1, e2]  →  Roll_ref
%
% The trained model and normalization parameters will be saved to:
%       bikeNN_outer.mat
%
% Requirements:
%   - Input dataset file: bike_outer_dataset.mat
%       containing variables:
%           X_all : [N × 2]   feature matrix
%           Y_all : [N × 1]   target vector (delta_ref from MPC)
%

    load('bike_outer_dataset.mat','X_all','Y_all');

    N = size(X_all,1);
    fprintf('Total samples: %d\n', N);

    %% 1. Shuffle dataset
        idx = randperm(N);
    X_all = X_all(idx,:);
    Y_all = Y_all(idx,:);

    %% 2. Split into train / validation / test sets
    Ntrain = floor(0.7 * N);
    Nval   = floor(0.15 * N);
    Ntest  = N - Ntrain - Nval;

    Xtrain = X_all(1:Ntrain, :);
    Ytrain = Y_all(1:Ntrain, :);

    Xval   = X_all(Ntrain+1 : Ntrain+Nval, :);
    Yval   = Y_all(Ntrain+1 : Ntrain+Nval, :);

    Xtest  = X_all(Ntrain+Nval+1 : end, :);
    Ytest  = Y_all(Ntrain+Nval+1 : end, :);

    %% ---------------------------------------------------------------
    % 3. Normalize input and output (critical for NN stability)
    %% ---------------------------------------------------------------
    muX    = mean(Xtrain,1);
    sigmaX = std(Xtrain, [], 1);
    sigmaX(sigmaX == 0) = 1;  % avoid division by zero

    muY    = mean(Ytrain,1);
    sigmaY = std(Ytrain, [], 1);
    sigmaY(sigmaY == 0) = 1;

    % Apply normalization
    Xtrain_n = (Xtrain - muX) ./ sigmaX;
    Xval_n   = (Xval   - muX) ./ sigmaX;

    Ytrain_n = (Ytrain - muY) ./ sigmaY;
    Yval_n   = (Yval   - muY) ./ sigmaY;

    %% 4. Define neural network architecture
    inputSize  = size(Xtrain_n, 2);  % should be 2
    outputSize = size(Ytrain_n, 2);  % should be 1

    layers = [
        featureInputLayer(inputSize, "Name", "input")
        fullyConnectedLayer(64, "Name", "fc1")
        reluLayer("Name", "relu1")
        fullyConnectedLayer(64, "Name", "fc2")
        reluLayer("Name", "relu2")
        fullyConnectedLayer(outputSize, "Name", "fc_out")
        regressionLayer("Name", "regout")];

    % Training options
    options = trainingOptions("adam", ...
        "MaxEpochs",           40, ...
        "MiniBatchSize",       1024, ...
        "InitialLearnRate",    1e-3, ...
        "ValidationData",      {Xval_n, Yval_n}, ...
        "ValidationFrequency", 50, ...
        "Shuffle",             "every-epoch", ...
        "Plots",               "training-progress", ...
        "Verbose",             false);

    %% 5. Train the network
    net = trainNetwork(Xtrain_n, Ytrain_n, layers, options);

    %% 6. Evaluate performance on test set
    Xtest_n = (Xtest - muX) ./ sigmaX;
    Ytest_n = (Ytest - muY) ./ sigmaY;

    Ypred_n = predict(net, Xtest_n);

    % Normalized MSE
    mse_test = mean((Ypred_n - Ytest_n).^2, "all");
    fprintf('Test normalized MSE: %.4e\n', mse_test);

    % De-normalized RMSE (physical units: radians)
    Ypred = Ypred_n .* sigmaY + muY;
    rmse = sqrt(mean((Ypred - Ytest).^2));
    fprintf('Test RMSE (delta_ref in rad): %.4e\n', rmse);

    %% 7. Save trained model
    save('bikeNN_outer.mat', 'net', 'muX', 'sigmaX', 'muY', 'sigmaY');

end
