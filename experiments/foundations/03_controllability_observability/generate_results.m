repoRoot = fileparts(fileparts(fileparts(fileparts(mfilename('fullpath')))));
figureDir = fullfile(repoRoot, 'figures', '03_controllability_observability');
generatedDir = fullfile(repoRoot, 'generated', '03_controllability_observability');

if ~exist(figureDir, 'dir')
    mkdir(figureDir);
end
if ~exist(generatedDir, 'dir')
    mkdir(generatedDir);
end

A = [0.0, 1.0; -2.0, -1.0];
B = [0.0; 1.0];
C = [1.0, 0.3];
x0Steer = [0.2; -0.8];
xfSteer = [1.0; 0.0];
x0Recon = [0.8; -1.1];
tHorizon = 5.0;
tGrid = linspace(0.0, tHorizon, 1200);
dt = tGrid(2) - tGrid(1);
reconstructionDt = 0.6;

gramian = zeros(2, 2);
for tau = tGrid
    factor = expm(A * (tHorizon - tau)) * B;
    gramian = gramian + factor * factor.' * dt;
end
phiHorizon = expm(A * tHorizon);
targetGap = xfSteer - phiHorizon * x0Steer;
control = zeros(size(tGrid));
gramianInv = inv(gramian);
for idx = 1:length(tGrid)
    tau = tGrid(idx);
    control(idx) = B.' * expm(A.' * (tHorizon - tau)) * gramianInv * targetGap;
end

[~, steerStates] = ode45(@(t, x) A * x + B * interp1(tGrid, control, t, 'linear', 'extrap'), tGrid, x0Steer);
steerStates = steerStates.';

Ad = expm(A * reconstructionDt);
observability = [C; C * Ad];
outputSamples = [C * x0Recon; C * Ad * x0Recon];
x0Estimate = observability \ outputSamples;

truthStates = zeros(2, length(tGrid));
estimateStates = zeros(2, length(tGrid));
for idx = 1:length(tGrid)
    truthStates(:, idx) = expm(A * tGrid(idx)) * x0Recon;
    estimateStates(:, idx) = expm(A * tGrid(idx)) * x0Estimate;
end
errorNorm = vecnorm(truthStates - estimateStates, 2, 1);

fig1 = figure('Visible', 'off', 'Color', 'w');
tiledlayout(2, 1, 'Padding', 'compact', 'TileSpacing', 'compact');
nexttile;
plot(tGrid, steerStates(1, :), 'LineWidth', 1.8, 'Color', [0.04, 0.37, 1.00]);
hold on;
plot(tGrid, steerStates(2, :), 'LineWidth', 1.8, 'Color', [0.82, 0.29, 0.36], 'LineStyle', '--');
ylabel('states');
title('Minimum-energy state steering');
grid on;
legend({'x_1(t)', 'x_2(t)'}, 'Location', 'northeast');

nexttile;
plot(tGrid, control, 'LineWidth', 1.8, 'Color', [0.16, 0.62, 0.56]);
xlabel('t (s)');
ylabel('u(t)');
grid on;
exportgraphics(fig1, fullfile(figureDir, 'minimum_energy_steering.png'), 'Resolution', 160);
close(fig1);

fig2 = figure('Visible', 'off', 'Color', 'w');
tiledlayout(3, 1, 'Padding', 'compact', 'TileSpacing', 'compact');
nexttile;
plot(tGrid, truthStates(1, :), 'LineWidth', 1.8, 'Color', [0.04, 0.37, 1.00]);
hold on;
plot(tGrid, estimateStates(1, :), 'LineWidth', 1.8, 'Color', [0.04, 0.37, 1.00], 'LineStyle', '--');
ylabel('x_1');
grid on;
legend({'True x_1', 'Reconstructed x_1'}, 'Location', 'northeast');

nexttile;
plot(tGrid, truthStates(2, :), 'LineWidth', 1.8, 'Color', [0.82, 0.29, 0.36]);
hold on;
plot(tGrid, estimateStates(2, :), 'LineWidth', 1.8, 'Color', [0.82, 0.29, 0.36], 'LineStyle', '--');
ylabel('x_2');
grid on;
legend({'True x_2', 'Reconstructed x_2'}, 'Location', 'northeast');

nexttile;
semilogy(tGrid, max(errorNorm, 1.0e-14), 'LineWidth', 1.8, 'Color', [0.16, 0.62, 0.56]);
xlabel('t (s)');
ylabel('||x-\hat x||_2');
grid on;
exportgraphics(fig2, fullfile(figureDir, 'state_reconstruction_comparison.png'), 'Resolution', 160);
close(fig2);

eigVals = eig(A);
controllability = [B, A * B];
report.system_matrix_A = A;
report.input_matrix_B = B;
report.output_matrix_C = C;
report.controllability_matrix = controllability;
report.controllability_rank = rank(controllability);
report.observability_matrix_from_two_samples = observability;
report.observability_rank = rank(observability);
report.stabilizable = rank(controllability) == 2;
report.detectable = rank(observability) == 2;
report.eigenvalues = [real(eigVals), imag(eigVals)];
report.minimum_energy_gramian = gramian;
report.steering_initial_state = x0Steer;
report.steering_target_state = xfSteer;
report.steering_terminal_error_norm = norm(steerStates(:, end) - xfSteer);
report.reconstruction_reference_state = x0Recon;
report.reconstruction_output_samples = outputSamples;
report.reconstructed_initial_state = x0Estimate;
report.reconstruction_error_norm = norm(x0Estimate - x0Recon);

fid = fopen(fullfile(generatedDir, 'controllability_observability_report_matlab.json'), 'w');
fprintf(fid, '%s', jsonencode(report, PrettyPrint=true));
fclose(fid);

fprintf('Saved MATLAB report to: %s\n', generatedDir);
fprintf('Saved MATLAB figures to: %s\n', figureDir);
