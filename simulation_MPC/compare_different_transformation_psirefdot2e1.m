% 定义矩阵 A 的函数形式
A_function = @(k1, k2) [
    -lr * V * k1 / (lr + lf), V - lr * V * k2 / (lr + lf);
    -V * k1 / (lr + lf), -V * k2 / (lr + lf)
];

% 初始矩阵 A
A_initial = A_function(k1, k2);

% 初始极点
initial_poles = eig(A_initial);
disp('初始极点：');
disp(initial_poles);

% ======================
% 情况 1：调整极点到 -1 ± j1
% ======================
target_poles_case1 = [-0.02 + 0.05j, -0.02 - 0.05j];

% 定义目标函数
objective_function = @(k, target_poles) eig(A_function(k(1), k(2))) - target_poles';

% 优化求解
disp('--- 情况 1：调整极点到 -0.15 ± j0.05 ---');
initial_guess = [k1, k2];
options = optimoptions('fsolve', 'Display', 'iter'); % 显示迭代过程
[k_optimal_case1, ~, ~] = fsolve(@(k) objective_function(k, target_poles_case1), initial_guess, options);

fprintf('情况 1 调整后的 k1 值为：%.4f\n', k_optimal_case1(1));
fprintf('情况 1 调整后的 k2 值为：%.4f\n', k_optimal_case1(2));

% ======================
% 情况 2：调整极点到 -2 ± j2
% ======================
target_poles_case2 = [-0.2 + 0.2j, -0.2 - 0.2j];

% 优化求解
disp('--- 情况 2：调整极点到 -0.2 ± j0.2 ---');
[k_optimal_case2, ~, ~] = fsolve(@(k) objective_function(k, target_poles_case2), initial_guess, options);

fprintf('情况 2 调整后的 k1 值为：%.4f\n', k_optimal_case2(1));
fprintf('情况 2 调整后的 k2 值为：%.4f\n', k_optimal_case2(2));

% ======================
% 验证和绘图
% ======================

% 验证新的 A 矩阵极点（两种情况）
A_new_case1 = A_function(k_optimal_case1(1), k_optimal_case1(2));
A_new_case2 = A_function(k_optimal_case2(1), k_optimal_case2(2));

new_poles_case1 = eig(A_new_case1);
new_poles_case2 = eig(A_new_case2);

disp('情况 1 的新极点：');
disp(new_poles_case1);

disp('情况 2 的新极点：');
disp(new_poles_case2);

% 创建状态空间系统并提取 G1(s)（两种情况）
transformation_B = [0; -1];
transformation_C = [1, 0; 0, 1];
transformation_D = [0; 0];

% 指定连续时间系统 (Ts = 0)
sys_initial = ss(A_initial, transformation_B, transformation_C, transformation_D, 0); % Ts = 0
sys_new_case1 = ss(A_new_case1, transformation_B, transformation_C, transformation_D, 0); % Ts = 0
sys_new_case2 = ss(A_new_case2, transformation_B, transformation_C, transformation_D, 0); % Ts = 0

G1_initial = tf(sys_initial(1));
G1_new_case1 = tf(sys_new_case1(1));
G1_new_case2 = tf(sys_new_case2(1));

% 使用 damp 函数计算阻尼比
[wn, zeta, poles] = damp(sys_initial) % 返回自然频率、阻尼比和极点
[wn, zeta, poles] = damp(sys_new_case1) % 返回自然频率、阻尼比和极点
[wn, zeta, poles] = damp(sys_new_case2); % 返回自然频率、阻尼比和极点

disp('情况 1 的 G1(s) 传递函数：');
G1_new_case1

disp('情况 2 的 G1(s) 传递函数：');
G1_new_case2

% 绘制 Bode 图
figure;
bode(G1_new_case1, 'b', G1_new_case2, 'r--');
grid on;
legend('G1(s) - 极点 -0.15 ± j0.05', 'G1(s) - 极点 -0.2 ± j0.2');
title('G1(s) 的 Bode 图比较');

% 绘制极点分布图
figure;
hold on;
plot(real(initial_poles), imag(initial_poles), 'ko', 'MarkerSize', 10, 'DisplayName', '初始极点');
plot(real(new_poles_case1), imag(new_poles_case1), 'bx', 'MarkerSize', 10, 'LineWidth', 2, 'DisplayName', '极点 -0.1 ± j0.1');
plot(real(new_poles_case2), imag(new_poles_case2), 'rx', 'MarkerSize', 10, 'LineWidth', 2, 'DisplayName', '极点 -0.2 ± j0.2');
grid on;
xlabel('实部');
ylabel('虚部');
title('极点分布图');
legend;
hold off;
