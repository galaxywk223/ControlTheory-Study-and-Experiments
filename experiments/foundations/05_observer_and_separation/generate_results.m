repoRoot = fileparts(fileparts(fileparts(fileparts(mfilename('fullpath')))));
figureDir = fullfile(repoRoot, 'figures', '05_observer_and_separation');
generatedDir = fullfile(repoRoot, 'generated', '05_observer_and_separation');

if ~exist(figureDir, 'dir')
    mkdir(figureDir);
end
if ~exist(generatedDir, 'dir')
    mkdir(generatedDir);
end

A = [0.0, 1.0; 1.2, 0.3];
B = [0.0; 1.0];
C = [1.0, 0.0];
x0 = [1.2; -0.8];
xHat0 = [0.0; 0.0];
controllerPoles = [-1.4, -2.2];
observerPoles = [-4.8, -5.6];
tEnd = 8.0;
tGrid = linspace(0.0, tEnd, 1200);

K = -place(A, B, controllerPoles);
L = place(A.', C.', observerPoles).';
Acl = A + B * K;
observerErrorMatrix = A - L * C;

[~, z] = ode45(@(t, z) augmented_dynamics(A, B, C, K, L, z), tGrid, [x0; xHat0]);
z = z.';
states = z(1:2, :);
estimates = z(3:4, :);
error = estimates - states;
errorNorm = vecnorm(error, 2, 1);
control = K * estimates;
combinedMatrix = [A, B * K; L * C, A + B * K - L * C];

fig1 = figure('Visible', 'off', 'Color', 'w');
tiledlayout(3, 1, 'Padding', 'compact', 'TileSpacing', 'compact');
nexttile;
plot(tGrid, states(1, :), 'LineWidth', 1.8, 'Color', [0.04, 0.37, 1.00]);
hold on;
plot(tGrid, estimates(1, :), 'LineWidth', 1.8, 'Color', [0.04, 0.37, 1.00], 'LineStyle', '--');
ylabel('x_1');
grid on;
legend({'True x_1', 'Estimated x_1'}, 'Location', 'northeast');

nexttile;
plot(tGrid, states(2, :), 'LineWidth', 1.8, 'Color', [0.82, 0.29, 0.36]);
hold on;
plot(tGrid, estimates(2, :), 'LineWidth', 1.8, 'Color', [0.82, 0.29, 0.36], 'LineStyle', '--');
ylabel('x_2');
grid on;
legend({'True x_2', 'Estimated x_2'}, 'Location', 'northeast');

nexttile;
semilogy(tGrid, max(errorNorm, 1.0e-14), 'LineWidth', 1.8, 'Color', [0.16, 0.62, 0.56]);
xlabel('t (s)');
ylabel('||e(t)||_2');
grid on;
exportgraphics(fig1, fullfile(figureDir, 'observer_state_estimates.png'), 'Resolution', 160);
close(fig1);

fig2 = figure('Visible', 'off', 'Color', 'w');
tiledlayout(2, 1, 'Padding', 'compact', 'TileSpacing', 'compact');
nexttile;
plot(tGrid, states(1, :), 'LineWidth', 1.8, 'Color', [0.04, 0.37, 1.00]);
hold on;
plot(tGrid, states(2, :), 'LineWidth', 1.8, 'Color', [0.82, 0.29, 0.36], 'LineStyle', '--');
ylabel('states');
title('Observer-based closed-loop response');
grid on;
legend({'x_1(t)', 'x_2(t)'}, 'Location', 'northeast');

nexttile;
plot(tGrid, control, 'LineWidth', 1.8, 'Color', [0.16, 0.62, 0.56]);
xlabel('t (s)');
ylabel('u(t)');
grid on;
exportgraphics(fig2, fullfile(figureDir, 'observer_closed_loop_response.png'), 'Resolution', 160);
close(fig2);

report.system_matrix_A = A;
report.input_matrix_B = B;
report.output_matrix_C = C;
report.state_feedback_gain_K = K;
report.observer_gain_L = L;
report.controller_poles = controllerPoles;
report.observer_poles = observerPoles;
report.state_feedback_closed_loop_eigenvalues = [real(eig(Acl)), imag(eig(Acl))];
report.observer_error_eigenvalues = [real(eig(observerErrorMatrix)), imag(eig(observerErrorMatrix))];
report.combined_augmented_eigenvalues = [real(eig(combinedMatrix)), imag(eig(combinedMatrix))];
report.initial_state = x0;
report.initial_estimate = xHat0;
report.max_estimation_error_norm = max(errorNorm);
report.final_estimation_error_norm = errorNorm(end);
report.control_peak_abs = max(abs(control));

fid = fopen(fullfile(generatedDir, 'observer_report_matlab.json'), 'w');
fprintf(fid, '%s', jsonencode(report, PrettyPrint=true));
fclose(fid);

fprintf('Saved MATLAB report to: %s\n', generatedDir);
fprintf('Saved MATLAB figures to: %s\n', figureDir);

function dz = augmented_dynamics(A, B, C, K, L, z)
    x = z(1:2);
    xHat = z(3:4);
    u = K * xHat;
    y = C * x;
    xDot = A * x + B * u;
    xHatDot = A * xHat + B * u + L * (y - C * xHat);
    dz = [xDot; xHatDot];
end
