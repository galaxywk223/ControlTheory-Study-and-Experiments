root = fileparts(fileparts(fileparts(fileparts(mfilename('fullpath')))));
fig_dir = fullfile(root, 'figures', '05_delay_neural_network_stability');
gen_dir = fullfile(root, 'generated', '05_delay_neural_network_stability');

if ~exist(fig_dir, 'dir')
    mkdir(fig_dir);
end
if ~exist(gen_dir, 'dir')
    mkdir(gen_dir);
end

A = -diag([1.0, 0.9]);
W = [0.35, -0.28; 0.22, 0.31];
L = eye(2);
history = [1.2; -1.0];

P1 = diag([1.47220303, 3.34527307]);
Q1 = diag([1.66769356, 1.72755548]);
P2 = diag([0.64768388, 3.40401467]);
Q2 = diag([0.49623577, 3.05849385]);
P3 = diag([2.50148372, 2.19018502]);
Q3 = diag([2.59316350, 1.49824932]);
R3 = diag([1.69318792, 0.66533231]);
U3 = diag([1.89660380, 2.26405287]);

M1 = [A' * P1 + P1 * A + Q1, P1 * W; (P1 * W)', -Q1];
M2 = [A' * P2 + P2 * A + L * Q2 * L, P2 * W; (P2 * W)', -Q2];
M3 = [
    A' * P3 + P3 * A, R3 * A + U3 * L, P3 * W;
    (R3 * A + U3 * L)', Q3 - (U3 + U3'), R3 * W;
    (P3 * W)', (R3 * W)', -Q3
];

theorem_1_margin = max(real(eig(M1)));
theorem_2_margin = max(real(eig(M2)));
theorem_3_margin = max(real(eig(M3)));

representative_taus = [0.2, 1.0, 2.0];
rep_time = cell(length(representative_taus), 1);
rep_states = cell(length(representative_taus), 1);
rep_peaks = zeros(length(representative_taus), 1);
rep_settling = zeros(length(representative_taus), 1);
rep_final = zeros(length(representative_taus), 2);

for i = 1:length(representative_taus)
    [time, states] = simulate_delay_network(A, W, representative_taus(i), history, 0.002, 20.0);
    rep_time{i} = time;
    rep_states{i} = states;
    rep_peaks(i) = max(abs(states), [], 'all');
    rep_settling(i) = settling_time(time, states, 1e-2);
    rep_final(i, :) = states(:, end)';
end

tau_grid = linspace(0.0, 2.2, 12);
settling_scan = zeros(length(tau_grid), 3);
for i = 1:length(tau_grid)
    [time, states] = simulate_delay_network(A, W, tau_grid(i), history, 0.002, 20.0);
    settling_scan(i, :) = [tau_grid(i), settling_time(time, states, 1e-2), norm(states(:, end))];
end

summary.A = A;
summary.W = W;
summary.history = history;
summary.activation = 'tanh';
summary.theorem_1.P = P1;
summary.theorem_1.Q = Q1;
summary.theorem_1.margin = theorem_1_margin;
summary.theorem_2.P = P2;
summary.theorem_2.Q = Q2;
summary.theorem_2.margin = theorem_2_margin;
summary.theorem_3.P = P3;
summary.theorem_3.Q = Q3;
summary.theorem_3.R = R3;
summary.theorem_3.U = U3;
summary.theorem_3.margin = theorem_3_margin;
summary.representative_taus = representative_taus;
summary.representative_peaks = rep_peaks;
summary.representative_settling = rep_settling;
summary.representative_final = rep_final;
summary.settling_scan = settling_scan;

fid = fopen(fullfile(gen_dir, 'summary.json'), 'w');
fwrite(fid, jsonencode(summary, PrettyPrint=true), 'char');
fclose(fid);

fid = fopen(fullfile(gen_dir, 'settling_scan.csv'), 'w');
fprintf(fid, 'tau,settling_time,final_norm\n');
for i = 1:size(settling_scan, 1)
    fprintf(fid, '%.6f,%.6f,%.6f\n', settling_scan(i, 1), settling_scan(i, 2), settling_scan(i, 3));
end
fclose(fid);

figure('Position', [120, 120, 840, 420], 'Color', 'w');
bar(categorical({'Theorem 1', 'Theorem 2', 'Theorem 3'}), [theorem_1_margin, theorem_2_margin, theorem_3_margin], ...
    'FaceColor', [0.29, 0.47, 0.69]);
hold on;
yline(0.0, '-', 'LineWidth', 1.1, 'Color', [0.15, 0.15, 0.15]);
ylabel('Largest eigenvalue');
title('LMI certificate margins');
grid on;
exportgraphics(gcf, fullfile(fig_dir, 'lmi_certificate_margins.png'), 'Resolution', 220);
close(gcf);

figure('Position', [140, 140, 920, 560], 'Color', 'w');
tiledlayout(2, 1, 'Padding', 'compact', 'TileSpacing', 'compact');

nexttile;
hold on;
for i = 1:length(representative_taus)
    plot(rep_time{i}, rep_states{i}(1, :), 'LineWidth', 1.8, 'DisplayName', sprintf('\\tau = %.1f', representative_taus(i)));
end
ylabel('x_1(t)');
title('State trajectories under different delays');
grid on;
legend('Location', 'northeast');

nexttile;
hold on;
for i = 1:length(representative_taus)
    plot(rep_time{i}, rep_states{i}(2, :), 'LineWidth', 1.8, 'DisplayName', sprintf('\\tau = %.1f', representative_taus(i)));
end
xlabel('Time (s)');
ylabel('x_2(t)');
grid on;
legend('Location', 'northeast');
exportgraphics(gcf, fullfile(fig_dir, 'state_trajectories_by_delay.png'), 'Resolution', 220);
close(gcf);

figure('Position', [160, 160, 860, 420], 'Color', 'w');
plot(settling_scan(:, 1), settling_scan(:, 2), '-o', 'LineWidth', 1.8, 'MarkerSize', 6);
xlabel('\tau');
ylabel('Settling time to ||x(t)|| <= 1e-2 (s)');
title('Delay versus settling time');
grid on;
exportgraphics(gcf, fullfile(fig_dir, 'delay_settling_time_scan.png'), 'Resolution', 220);
close(gcf);

[time_phase, states_phase] = simulate_delay_network(A, W, 2.0, history, 0.002, 20.0);
figure('Position', [180, 180, 560, 520], 'Color', 'w');
plot(states_phase(1, :), states_phase(2, :), 'LineWidth', 1.8, 'Color', [0.49, 0.36, 0.65]);
hold on;
scatter(states_phase(1, 1), states_phase(2, 1), 42, [0.74, 0.26, 0.17], 'filled');
scatter(states_phase(1, end), states_phase(2, end), 42, [0.17, 0.42, 0.69], 'filled');
xlabel('x_1(t)');
ylabel('x_2(t)');
title('Phase trajectory for \tau = 2.0');
grid on;
exportgraphics(gcf, fullfile(fig_dir, 'phase_portrait_tau_2_0.png'), 'Resolution', 220);
close(gcf);

fprintf('Delay neural network reproduction completed.\n');
fprintf('Theorem 1 margin: %.6f\n', theorem_1_margin);
fprintf('Theorem 2 margin: %.6f\n', theorem_2_margin);
fprintf('Theorem 3 margin: %.6f\n', theorem_3_margin);

function [time, states] = simulate_delay_network(A, W, tau, history, dt, horizon)
    delay_steps = max(0, round(tau / dt));
    total_steps = round(horizon / dt);

    if delay_steps == 0
        time = linspace(0.0, horizon, total_steps + 1);
        states = zeros(2, total_steps + 1);
        states(:, 1) = history;
        for k = 1:total_steps
            delayed_state = states(:, k);
            derivative = A * states(:, k) + W * tanh(delayed_state);
            states(:, k + 1) = states(:, k) + dt * derivative;
        end
        return;
    end

    tau_eff = delay_steps * dt;
    time_full = linspace(-tau_eff, horizon, total_steps + delay_steps + 1);
    states_full = zeros(2, length(time_full));
    states_full(:, 1:(delay_steps + 1)) = repmat(history, 1, delay_steps + 1);

    for k = (delay_steps + 1):(length(time_full) - 1)
        delayed_state = states_full(:, k - delay_steps);
        derivative = A * states_full(:, k) + W * tanh(delayed_state);
        states_full(:, k + 1) = states_full(:, k) + dt * derivative;
    end

    time = time_full((delay_steps + 1):end);
    states = states_full(:, (delay_steps + 1):end);
end

function value = settling_time(time, states, threshold)
    norm_series = vecnorm(states, 2, 1);
    suffix_max = flip(cummax(flip(norm_series)));
    idx = find(suffix_max <= threshold, 1, 'first');
    if isempty(idx)
        value = NaN;
    else
        value = time(idx);
    end
end
