repoRoot = fileparts(fileparts(fileparts(mfilename('fullpath'))));
figureDir = fullfile(repoRoot, 'figures', '03_periodic_sampling');
generatedDir = fullfile(repoRoot, 'generated', '03_periodic_sampling');

if ~exist(figureDir, 'dir')
    mkdir(figureDir);
end

if ~exist(generatedDir, 'dir')
    mkdir(generatedDir);
end

A = [-0.479908, -3.81625; 5.1546, 14.4723];
B = diag([5.8705212, 15.50107]);
K = [-0.3399, -0.0263; -0.0628, -1.2527];
alpha = 0.05;
h = 0.125;
x0 = [1; -1];
tEnd = 6.0;
pointsPerInterval = 40;

numIntervals = round(tEnd / h);
sampleTimes = linspace(0, numIntervals * h, numIntervals + 1);
sampledStates = zeros(numIntervals + 1, 2);
sampledStates(1, :) = x0.';
sampledControls = zeros(numIntervals, 2);

localTau = linspace(0, h, pointsPerInterval + 1);
localTau(end) = [];
continuousTimes = [];
continuousStates = [];

    function [Ad, Bd] = zohDiscretize(localA, localB, tau)
        n = size(localA, 1);
        m = size(localB, 2);
        augmented = zeros(n + m, n + m);
        augmented(1:n, 1:n) = localA;
        augmented(1:n, n+1:n+m) = localB;
        transition = expm(augmented * tau);
        Ad = transition(1:n, 1:n);
        Bd = transition(1:n, n+1:n+m);
    end

[Ad_h, Bd_h] = zohDiscretize(A, B, h);

localAd = cell(length(localTau), 1);
localBd = cell(length(localTau), 1);
for idx = 1:length(localTau)
    [localAd{idx}, localBd{idx}] = zohDiscretize(A, B, localTau(idx));
end

for k = 1:numIntervals
    xk = sampledStates(k, :).';
    uk = K * xk;
    sampledControls(k, :) = uk.';

    t0 = sampleTimes(k);
    localTimes = t0 + localTau;
    localStates = zeros(length(localTau), 2);
    for idx = 1:length(localTau)
        localStates(idx, :) = (localAd{idx} * xk + localBd{idx} * uk).';
    end

    continuousTimes = [continuousTimes, localTimes]; %#ok<AGROW>
    continuousStates = [continuousStates; localStates]; %#ok<AGROW>

    sampledStates(k + 1, :) = (Ad_h * xk + Bd_h * uk).';
end

continuousTimes = [continuousTimes, sampleTimes(end)];
continuousStates = [continuousStates; sampledStates(end, :)];

discreteClosedLoop = Ad_h + Bd_h * K;
discreteEig = eig(discreteClosedLoop);
rho = max(abs(discreteEig));

fig1 = figure('Visible', 'off', 'Color', 'w');
plot(continuousTimes, continuousStates(:, 1), 'LineWidth', 1.8, 'Color', [0.12, 0.47, 0.71]);
hold on;
plot(continuousTimes, continuousStates(:, 2), 'LineWidth', 1.8, 'Color', [0.82, 0.41, 0.12]);
scatter(sampleTimes, sampledStates(:, 1), 16, [0.85, 0.65, 0.13], 'filled');
scatter(sampleTimes, sampledStates(:, 2), 14, [0.48, 0.17, 0.74], 'filled');
yline(0, 'k-', 'LineWidth', 0.6);
xlabel('t (s)');
ylabel('states');
title(sprintf('Continuous & sampled states (h=%.3f s, rho=%.6f)', h, rho));
grid on;
legend({'x_1(t)', 'x_2(t)', 'x_1[k]', 'x_2[k]'}, 'Location', 'southeast');
exportgraphics(fig1, fullfile(figureDir, 'continuous_sampled_states.png'), 'Resolution', 160);
close(fig1);

controlTimes = sampleTimes;
controlValues = [sampledControls; sampledControls(end, :)];

fig2 = figure('Visible', 'off', 'Color', 'w');
stairs(controlTimes, controlValues(:, 1), 'LineWidth', 1.8, 'Color', [0.12, 0.47, 0.71]);
hold on;
stairs(controlTimes, controlValues(:, 2), 'LineWidth', 1.8, 'Color', [0.82, 0.41, 0.12]);
xlabel('t (s)');
ylabel('u(t)');
title('ZOH control signals');
grid on;
legend({'u_1(t)', 'u_2(t)'}, 'Location', 'southeast');
exportgraphics(fig2, fullfile(figureDir, 'zoh_control_signals.png'), 'Resolution', 160);
close(fig2);

report = struct();
report.alpha = alpha;
report.sampling_period = h;
report.system_matrix_A = A;
report.input_matrix_B = B;
report.feedback_gain_K = K;
report.initial_state = x0.';
report.discrete_closed_loop_matrix = discreteClosedLoop;
report.discrete_closed_loop_eigenvalues = [real(discreteEig), imag(discreteEig)];
report.spectral_radius = rho;
report.final_sampled_state = sampledStates(end, :);
report.final_control = sampledControls(end, :);

reportPath = fullfile(generatedDir, 'periodic_sampling_report.json');
fid = fopen(reportPath, 'w');
fprintf(fid, '%s', jsonencode(report, PrettyPrint=true));
fclose(fid);

fprintf('Saved MATLAB report to: %s\n', generatedDir);
fprintf('Saved MATLAB figures to: %s\n', figureDir);
