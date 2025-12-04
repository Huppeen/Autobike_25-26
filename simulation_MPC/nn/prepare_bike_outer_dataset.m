function prepare_bike_outer_dataset()
% Input:
%    training_data.mat
%
% Output:
%    Saves 'bike_outer_dataset.mat' containing:
%      X_all: [N × 2] feature matrix  (col1=e1, col2=e2)
%      Y_all: [N × 1] labels (roll_ref)

    % === Load MAT file ===
    tmp = load('training_data.mat');   % 结构体
    M   = tmp.Training_data;           % 真正的数据矩阵 62000×3

    % Extract required columns
    e1       = M(:,1);                 % col1
    e2       = M(:,2);                 % col2
    roll_ref = M(:,3);                 % col3, label

    % Basic cleaning
    valid = ~(isnan(e1) | isnan(e2) | isnan(roll_ref) | ...
              isinf(e1) | isinf(e2) | isinf(roll_ref));

    e1       = e1(valid);
    e2       = e2(valid);
    roll_ref = roll_ref(valid);

    % Remove initial unstable samples
    dropN = 50;
    if length(e1) > dropN
        e1       = e1(dropN+1:end);
        e2       = e2(dropN+1:end);
        roll_ref = roll_ref(dropN+1:end);
    end

    % Assemble dataset
    X_all = [e1, e2];
    Y_all = roll_ref;

    fprintf('Prepared dataset with %d samples.\n', size(X_all,1));

    save('bike_outer_dataset.mat','X_all','Y_all');
end
