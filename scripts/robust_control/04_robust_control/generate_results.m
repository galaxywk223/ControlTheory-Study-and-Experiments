root = fileparts(fileparts(fileparts(fileparts(mfilename('fullpath')))));
fig_dir = fullfile(root, 'figures', '04_robust_control');
gen_dir = fullfile(root, 'generated', '04_robust_control');

if ~exist(fig_dir, 'dir')
    mkdir(fig_dir);
end
if ~exist(gen_dir, 'dir')
    mkdir(gen_dir);
end

B = [0; 1];
D = [0; 1];
E = diag([1.0, 0.4]);
Q = diag([6.0, 2.0]);
R = 1.0;

A_nominal = uncertainty_matrix(0.0, 0.0);
[P, ~, K_lqr] = care(A_nominal, B, Q, R);
K = -K_lqr;

rho_1_grid = linspace(-0.6, 0.6, 81);
rho_2_grid = linspace(-0.25, 0.25, 81);
open_loop_alpha = zeros(length(rho_2_grid), length(rho_1_grid));
closed_loop_alpha = zeros(length(rho_2_grid), length(rho_1_grid));

for i = 1:length(rho_2_grid)
    for j = 1:length(rho_1_grid)
        A = uncertainty_matrix(rho_1_grid(j), rho_2_grid(i));
        A_cl = A + B * K;
        open_loop_alpha(i, j) = max(real(eig(A)));
        closed_loop_alpha(i, j) = max(real(eig(A_cl)));
    end
end

vertex_points = {
    'V1', -0.6, -0.25;
    'V2', -0.6, 0.25;
    'V3',  0.6, -0.25;
    'V4',  0.6, 0.25;
    'Nominal', 0.0, 0.0
};
frequency_grid = logspace(-3, 3, 4000);
vertex_metrics = zeros(size(vertex_points, 1), 6);
vertex_labels = strings(size(vertex_points, 1), 1);

for i = 1:size(vertex_points, 1)
    vertex_labels(i) = string(vertex_points{i, 1});
    rho_1 = vertex_points{i, 2};
    rho_2 = vertex_points{i, 3};
    A = uncertainty_matrix(rho_1, rho_2);
    A_cl = A + B * K;
    residual = A_cl' * P + P * A_cl;
    vertex_metrics(i, :) = [
        rho_1, ...
        rho_2, ...
        max(real(eig(A))), ...
        max(real(eig(A_cl))), ...
        max(real(eig(residual))), ...
        estimate_hinf_norm(A_cl, D, E, frequency_grid)
    ];
end

representative_cases = {
    'rho1=-0.6, rho2=-0.25', -0.6, -0.25;
    'rho1=0.0, rho2=0.0', 0.0, 0.0;
    'rho1=0.6, rho2=0.25', 0.6, 0.25
};
case_count = size(representative_cases, 1);
response_data = cell(case_count, 4);
peak_table = zeros(case_count, 2);

for i = 1:case_count
    label = string(representative_cases{i, 1});
    rho_1 = representative_cases{i, 2};
    rho_2 = representative_cases{i, 3};
    A_cl = uncertainty_matrix(rho_1, rho_2) + B * K;
    [time, states] = ode45(@(t, x) A_cl * x + [0; disturbance_signal(t)], [0, 15], [0; 0]);
    controls = (K * states')';
    response_data{i, 1} = label;
    response_data{i, 2} = time;
    response_data{i, 3} = states;
    response_data{i, 4} = controls;
    peak_table(i, :) = [max(abs(states), [], 'all'), max(abs(controls), [], 'all')];
end

summary.A_nominal = A_nominal;
summary.B = B;
summary.D = D;
summary.E = E;
summary.K = K;
summary.P = P;
summary.uncertainty_box.rho_1 = [-0.6, 0.6];
summary.uncertainty_box.rho_2 = [-0.25, 0.25];
summary.worst_closed_loop_alpha_on_grid = max(closed_loop_alpha, [], 'all');
summary.best_closed_loop_alpha_on_grid = min(closed_loop_alpha, [], 'all');
summary.worst_open_loop_alpha_on_grid = max(open_loop_alpha, [], 'all');
summary.best_open_loop_alpha_on_grid = min(open_loop_alpha, [], 'all');
summary.vertex_labels = vertex_labels;
summary.vertex_metrics = vertex_metrics;
summary.representative_case_labels = string(representative_cases(:, 1));
summary.representative_peaks = peak_table;

summary_json = jsonencode(summary, PrettyPrint=true);
fid = fopen(fullfile(gen_dir, 'summary.json'), 'w');
fwrite(fid, summary_json, 'char');
fclose(fid);

metric_headers = {'label', 'rho_1', 'rho_2', 'open_loop_alpha', 'closed_loop_alpha', ...
    'lyapunov_residual_alpha', 'hinf_estimate'};
fid = fopen(fullfile(gen_dir, 'vertex_metrics.csv'), 'w');
fprintf(fid, '%s,%s,%s,%s,%s,%s,%s\n', metric_headers{:});
for i = 1:size(vertex_metrics, 1)
    fprintf( ...
        fid, ...
        '%s,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n', ...
        char(vertex_labels(i)), ...
        vertex_metrics(i, 1), ...
        vertex_metrics(i, 2), ...
        vertex_metrics(i, 3), ...
        vertex_metrics(i, 4), ...
        vertex_metrics(i, 5), ...
        vertex_metrics(i, 6) ...
    );
end
fclose(fid);

figure('Position', [120, 120, 1180, 420], 'Color', 'w');
tiledlayout(1, 2, 'Padding', 'compact', 'TileSpacing', 'compact');

nexttile;
imagesc(rho_1_grid, rho_2_grid, open_loop_alpha);
set(gca, 'YDir', 'normal');
colorbar;
xlabel('\rho_1');
ylabel('\rho_2');
title('Open-loop spectral abscissa');

nexttile;
imagesc(rho_1_grid, rho_2_grid, closed_loop_alpha);
set(gca, 'YDir', 'normal');
colorbar;
xlabel('\rho_1');
ylabel('\rho_2');
title('Closed-loop spectral abscissa');
exportgraphics(gcf, fullfile(fig_dir, 'spectral_abscissa_scan.png'), 'Resolution', 220);
close(gcf);

figure('Position', [150, 150, 840, 420], 'Color', 'w');
bar(categorical(vertex_labels), vertex_metrics(:, 6), 'FaceColor', [0.29, 0.47, 0.69]);
hold on;
yline(0.5, '--', '\gamma = 0.5', 'LineWidth', 1.4, 'Color', [0.74, 0.26, 0.17]);
ylabel('Estimated gain');
title('Estimated H_{\infty} gain at interval vertices');
grid on;
exportgraphics(gcf, fullfile(fig_dir, 'vertex_hinf_estimates.png'), 'Resolution', 220);
close(gcf);

figure('Position', [160, 140, 920, 560], 'Color', 'w');
tiledlayout(2, 1, 'Padding', 'compact', 'TileSpacing', 'compact');

nexttile;
hold on;
for i = 1:case_count
    plot(response_data{i, 2}, response_data{i, 3}(:, 1), 'LineWidth', 1.7, 'DisplayName', response_data{i, 1} + " : x_1");
end
ylabel('x_1(t)');
title('Closed-loop states under decaying disturbance');
grid on;
legend('Location', 'northeastoutside');

nexttile;
hold on;
for i = 1:case_count
    plot(response_data{i, 2}, response_data{i, 3}(:, 2), 'LineWidth', 1.7, 'DisplayName', response_data{i, 1} + " : x_2");
end
xlabel('Time (s)');
ylabel('x_2(t)');
grid on;
legend('Location', 'northeastoutside');
exportgraphics(gcf, fullfile(fig_dir, 'state_response_under_disturbance.png'), 'Resolution', 220);
close(gcf);

figure('Position', [180, 160, 920, 420], 'Color', 'w');
hold on;
for i = 1:case_count
    plot(response_data{i, 2}, response_data{i, 4}, 'LineWidth', 1.8, 'DisplayName', response_data{i, 1});
end
time_plot = linspace(0, 15, 1501);
plot(time_plot, disturbance_signal(time_plot), '--', 'LineWidth', 1.2, 'Color', [0.15, 0.15, 0.15], 'DisplayName', 'w(t)');
xlabel('Time (s)');
ylabel('Signal value');
title('Control input and disturbance');
grid on;
legend('Location', 'northeastoutside');
exportgraphics(gcf, fullfile(fig_dir, 'control_response_under_disturbance.png'), 'Resolution', 220);
close(gcf);

fprintf('Robust control reproduction completed.\n');
fprintf('K = [%.6f %.6f]\n', K(1), K(2));
fprintf('Worst closed-loop spectral abscissa on grid: %.6f\n', max(closed_loop_alpha, [], 'all'));
fprintf('Worst estimated vertex Hinf gain: %.6f\n', max(vertex_metrics(:, 6)));

function A = uncertainty_matrix(rho_1, rho_2)
    A = [0.0, 1.0; 1.2 + rho_1, 0.3 + rho_2];
end

function value = disturbance_signal(t)
    value = 0.8 .* exp(-0.3 .* t) .* sin(2.4 .* t);
end

function gain = estimate_hinf_norm(A_cl, D, E, frequencies)
    gain = 0.0;
    identity = eye(size(A_cl));
    for idx = 1:length(frequencies)
        omega = frequencies(idx);
        transfer = E * ((1i * omega * identity - A_cl) \ D);
        sigma = norm(transfer, 2);
        gain = max(gain, sigma);
    end
end
