repoRoot = fileparts(fileparts(fileparts(fileparts(mfilename('fullpath')))));
figureDir = fullfile(repoRoot, 'figures', '01_lti_stability');
generatedDir = fullfile(repoRoot, 'generated', '01_lti_stability');

if ~exist(figureDir, 'dir')
    mkdir(figureDir);
end

if ~exist(generatedDir, 'dir')
    mkdir(generatedDir);
end

A = [0, 1; -1, -1];
Q = eye(2);
tEnd = 20;
numSamples = 1000;
tGrid = linspace(0, tEnd, numSamples);
x0 = [1; -1];

eigenvalues = eig(A);

% Solve A'P + P A = -I without relying on extra toolboxes.
linearSystem = kron(eye(size(A)), A') + kron(A', eye(size(A)));
vecP = linearSystem \ (-Q(:));
P = reshape(vecP, size(A));
P = (P + P') / 2;

states = zeros(numSamples, 2);
for idx = 1:numSamples
    states(idx, :) = (expm(A * tGrid(idx)) * x0).';
end

fig1 = figure('Visible', 'off', 'Color', 'w');
subplot(2, 1, 1);
plot(tGrid, states(:, 1), 'LineWidth', 2, 'Color', [0.04, 0.37, 1.0]);
grid on;
ylabel('$x_1(t)$', 'Interpreter', 'latex');
title('State Trajectories for A = [[0, 1], [-1, -1]] and x(0) = [1, -1]^T');

subplot(2, 1, 2);
plot(tGrid, states(:, 2), 'LineWidth', 2, 'Color', [0.82, 0.29, 0.36]);
grid on;
xlabel('t');
ylabel('$x_2(t)$', 'Interpreter', 'latex');

exportgraphics(fig1, fullfile(figureDir, 'state_trajectories.png'), 'Resolution', 160);
close(fig1);

gridRange = linspace(-2, 2, 21);
[x1Grid, x2Grid] = meshgrid(gridRange, gridRange);
u = A(1, 1) * x1Grid + A(1, 2) * x2Grid;
v = A(2, 1) * x1Grid + A(2, 2) * x2Grid;

initialConditions = [
    1.0, -1.0;
    1.2, 0.8;
   -1.0, 1.0;
   -1.5, -0.5;
    0.5, -1.5
];

fig2 = figure('Visible', 'off', 'Color', 'w');
quiver(x1Grid, x2Grid, u, v, 'Color', [0.72, 0.78, 0.86], 'AutoScale', 'on');
hold on;

for idx = 1:size(initialConditions, 1)
    xInit = initialConditions(idx, :).';
    trajectory = zeros(numSamples, 2);
    for tIdx = 1:numSamples
        trajectory(tIdx, :) = (expm(A * tGrid(tIdx)) * xInit).';
    end
    plot(trajectory(:, 1), trajectory(:, 2), 'LineWidth', 2);
    scatter(trajectory(1, 1), trajectory(1, 2), 20, 'filled');
end

scatter(0, 0, 30, 'k', 'filled');
grid on;
axis equal;
xlim([-2, 2]);
ylim([-2, 2]);
xlabel('$x_1$', 'Interpreter', 'latex');
ylabel('$x_2$', 'Interpreter', 'latex');
title('Phase Portrait of the LTI System');

exportgraphics(fig2, fullfile(figureDir, 'phase_portrait.png'), 'Resolution', 160);
close(fig2);

residual = A' * P + P * A + Q;

report = struct();
report.system_matrix = A;
report.system_eigenvalues = [real(eigenvalues), imag(eigenvalues)];
report.reference_initial_condition = x0.';
report.lyapunov_equation = 'A^T P + P A = -I';
report.lyapunov_matrix = P;
report.lyapunov_matrix_eigenvalues = eig(P).';
report.lyapunov_residual_frobenius_norm = norm(residual, 'fro');
report.final_state_at_t_end = states(end, :);

reportPath = fullfile(generatedDir, 'lyapunov_report.json');
fid = fopen(reportPath, 'w');
fprintf(fid, '%s', jsonencode(report, PrettyPrint=true));
fclose(fid);

fprintf('Saved MATLAB report to: %s\n', generatedDir);
fprintf('Saved MATLAB figures to: %s\n', figureDir);
