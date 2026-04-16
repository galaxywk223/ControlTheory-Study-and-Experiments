repoRoot = fileparts(fileparts(fileparts(fileparts(mfilename('fullpath')))));
figureDir = fullfile(repoRoot, 'figures', '06_optimal_control');
generatedDir = fullfile(repoRoot, 'generated', '06_optimal_control');

if ~exist(figureDir, 'dir')
    mkdir(figureDir);
end
if ~exist(generatedDir, 'dir')
    mkdir(generatedDir);
end

A = [0.0, 1.0; 1.0, 0.2];
B = [0.0; 1.0];
x0 = [1.0; -1.5];
tGrid = linspace(0.0, 10.0, 1200);

cases(1).name = 'state_priority';
cases(1).label = 'State-priority';
cases(1).Q = diag([12.0, 2.0]);
cases(1).R = 0.25;
cases(1).color = [0.04, 0.37, 1.00];

cases(2).name = 'control_priority';
cases(2).label = 'Control-priority';
cases(2).Q = diag([3.0, 1.0]);
cases(2).R = 1.50;
cases(2).color = [0.82, 0.29, 0.36];

fig1 = figure('Visible', 'off', 'Color', 'w');
tiledlayout(2, 1, 'Padding', 'compact', 'TileSpacing', 'compact');

fig2 = figure('Visible', 'off', 'Color', 'w');
hold on;

report.system_matrix_A = A;
report.input_matrix_B = B;
report.initial_state = x0;

for idx = 1:length(cases)
    Q = cases(idx).Q;
    R = cases(idx).R;
    P = care(A, B, Q, R);
    K = -(R \ (B.' * P));
    Acl = A + B * K;
    states = zeros(2, length(tGrid));
    control = zeros(1, length(tGrid));
    for k = 1:length(tGrid)
        states(:, k) = expm(Acl * tGrid(k)) * x0;
        control(k) = K * states(:, k);
    end
    integrand = zeros(1, length(tGrid));
    for k = 1:length(tGrid)
        integrand(k) = states(:, k).' * Q * states(:, k) + R * control(k)^2;
    end

    figure(fig1);
    nexttile(1);
    plot(tGrid, states(1, :), 'LineWidth', 1.8, 'Color', cases(idx).color, 'DisplayName', [cases(idx).label ' x_1']);
    hold on;
    ylabel('x_1(t)');
    title('LQR state responses under different weight selections');
    grid on;

    nexttile(2);
    lineStyle = '-';
    if idx == 2
        lineStyle = '--';
    end
    plot(tGrid, states(2, :), 'LineWidth', 1.8, 'Color', cases(idx).color, 'LineStyle', lineStyle, 'DisplayName', [cases(idx).label ' x_2']);
    hold on;
    ylabel('x_2(t)');
    xlabel('t (s)');
    grid on;

    figure(fig2);
    plot(tGrid, control, 'LineWidth', 1.8, 'Color', cases(idx).color, 'DisplayName', cases(idx).label);

    report.cases.(cases(idx).name).Q = Q;
    report.cases.(cases(idx).name).R = R;
    report.cases.(cases(idx).name).P = P;
    report.cases.(cases(idx).name).K = K;
    report.cases.(cases(idx).name).closed_loop_eigenvalues = [real(eig(Acl)), imag(eig(Acl))];
    report.cases.(cases(idx).name).state_peak_abs = max(abs(states), [], 'all');
    report.cases.(cases(idx).name).control_peak_abs = max(abs(control));
    report.cases.(cases(idx).name).quadratic_cost = trapz(tGrid, integrand);
    report.cases.(cases(idx).name).final_state = states(:, end);
end

figure(fig1);
nexttile(1); legend('Location', 'northeast');
nexttile(2); legend('Location', 'northeast');
exportgraphics(fig1, fullfile(figureDir, 'lqr_weight_tradeoff_states.png'), 'Resolution', 160);
close(fig1);

figure(fig2);
xlabel('t (s)');
ylabel('u(t)');
title('LQR control effort trade-off');
grid on;
legend('Location', 'northeast');
exportgraphics(fig2, fullfile(figureDir, 'lqr_weight_tradeoff_control.png'), 'Resolution', 160);
close(fig2);

fid = fopen(fullfile(generatedDir, 'lqr_report_matlab.json'), 'w');
fprintf(fid, '%s', jsonencode(report, PrettyPrint=true));
fclose(fid);

fprintf('Saved MATLAB report to: %s\n', generatedDir);
fprintf('Saved MATLAB figures to: %s\n', figureDir);
