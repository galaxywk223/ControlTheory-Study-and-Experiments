repoRoot = fileparts(fileparts(fileparts(fileparts(mfilename('fullpath')))));
figureDir = fullfile(repoRoot, 'figures', '01_state_space_modeling');
generatedDir = fullfile(repoRoot, 'generated', '01_state_space_modeling');

if ~exist(figureDir, 'dir')
    mkdir(figureDir);
end
if ~exist(generatedDir, 'dir')
    mkdir(generatedDir);
end

m = 1.0;
c = 0.6;
k = 2.0;
A = [0.0, 1.0; -k / m, -c / m];
B = [0.0; 1.0 / m];
C = [1.0, 0.0];
D = 0.0;
x0 = [0.8; -0.3];
tEnd = 12.0;
tGrid = linspace(0.0, tEnd, 1200);

[freeStates, freeOutputs] = simulate_response(A, B, C, D, x0, @(t) 0.0, tGrid);
[~, zeroOutputs] = simulate_response(A, B, C, D, [0.0; 0.0], @(t) 0.0, tGrid);
[forcedStates, forcedOutputs] = simulate_response(A, B, C, D, [0.0; 0.0], @(t) 1.0, tGrid);

equilibriumState = -A \ B;
equilibriumOutput = C * equilibriumState;
transitionAt1s = expm(A);
eigVals = eig(A);

fig1 = figure('Visible', 'off', 'Color', 'w');
tiledlayout(2, 1, 'Padding', 'compact', 'TileSpacing', 'compact');
nexttile;
plot(tGrid, freeStates(1, :), 'LineWidth', 1.8, 'Color', [0.04, 0.37, 1.00]);
hold on;
plot(tGrid, freeStates(2, :), 'LineWidth', 1.8, 'Color', [0.82, 0.29, 0.36], 'LineStyle', '--');
ylabel('states');
title('State-space response of the mass-spring-damper model');
grid on;
legend({'x_1(t)=q(t)', 'x_2(t)=dq/dt'}, 'Location', 'northeast');

nexttile;
plot(tGrid, freeOutputs, 'LineWidth', 1.8, 'Color', [0.16, 0.62, 0.56]);
xlabel('t (s)');
ylabel('output');
grid on;
legend({'y(t)=q(t)'}, 'Location', 'northeast');
exportgraphics(fig1, fullfile(figureDir, 'state_output_response.png'), 'Resolution', 160);
close(fig1);

fig2 = figure('Visible', 'off', 'Color', 'w');
plot(tGrid, zeroOutputs, 'LineWidth', 1.8, 'Color', [0.30, 0.43, 0.96]);
hold on;
plot(tGrid, forcedOutputs, 'LineWidth', 1.8, 'Color', [0.91, 0.44, 0.32]);
yline(equilibriumOutput, '--', 'Color', [0.12, 0.12, 0.12], 'LineWidth', 1.0);
xlabel('t (s)');
ylabel('y(t)');
title('Forced-response comparison in state-space form');
grid on;
legend({'Zero input', 'Unit input', 'Unit-input equilibrium'}, 'Location', 'southeast');
exportgraphics(fig2, fullfile(figureDir, 'forced_response_comparison.png'), 'Resolution', 160);
close(fig2);

report.physical_parameters.mass = m;
report.physical_parameters.damping = c;
report.physical_parameters.stiffness = k;
report.state_space_model.A = A;
report.state_space_model.B = B;
report.state_space_model.C = C;
report.state_space_model.D = D;
report.reference_initial_state = x0;
report.unit_input_equilibrium_state = equilibriumState;
report.unit_input_equilibrium_output = equilibriumOutput;
report.transition_matrix_at_1s = transitionAt1s;
report.eigenvalues = [real(eigVals), imag(eigVals)];
report.free_response_final_state = freeStates(:, end);
report.free_response_final_output = freeOutputs(end);
report.forced_response_final_state = forcedStates(:, end);
report.forced_response_final_output = forcedOutputs(end);

fid = fopen(fullfile(generatedDir, 'model_report_matlab.json'), 'w');
fprintf(fid, '%s', jsonencode(report, PrettyPrint=true));
fclose(fid);

fprintf('Saved MATLAB report to: %s\n', generatedDir);
fprintf('Saved MATLAB figures to: %s\n', figureDir);

function [states, outputs] = simulate_response(A, B, C, D, x0, inputSignal, tGrid)
    [~, states] = ode45(@(t, x) A * x + B * inputSignal(t), tGrid, x0);
    controls = arrayfun(inputSignal, tGrid);
    outputs = (C * states.').' + D * controls;
    states = states.';
end
