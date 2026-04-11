repoRoot = fileparts(fileparts(fileparts(mfilename('fullpath'))));
figureDir = fullfile(repoRoot, 'figures', '02_lti_control');
generatedDir = fullfile(repoRoot, 'generated', '02_lti_control');

if ~exist(figureDir, 'dir')
    mkdir(figureDir);
end

if ~exist(generatedDir, 'dir')
    mkdir(generatedDir);
end

A = [0.12, 0.08; -0.18, -0.02];
B = eye(2);
C = [1, 0; 0, 0.8];
x0 = [1; 2];
tEnd = 20;
numSamples = 1000;
tGrid = linspace(0, tEnd, numSamples);

stateFeedbackGains = struct();
stateFeedbackGains.theorem_2 = [-2.12, 17.92; -17.82, -2.28];
stateFeedbackGains.theorem_3 = [-1.92, 4.42; -5.82, -1.58];

outputFeedbackGains = struct();
outputFeedbackGains.theorem_5 = [-2.32, 21.775; -16.82, -2.475];
outputFeedbackGains.theorem_6 = [-2.92, 0.65; -1.62, -4.35];
outputFeedbackGains.theorem_7 = [-2.52, 1.65; -2.42, -3.725];

figureNames = struct();
figureNames.open_loop = 'open_loop_state_response.png';
figureNames.theorem_2 = 'theorem_2_state_feedback_response.png';
figureNames.theorem_3 = 'theorem_3_state_feedback_response.png';
figureNames.theorem_5 = 'theorem_5_output_feedback_response.png';
figureNames.theorem_6 = 'theorem_6_output_feedback_response.png';
figureNames.theorem_7 = 'theorem_7_output_feedback_response.png';

report = struct();
report.system_matrix_A = A;
report.input_matrix_B = B;
report.output_matrix_C = C;
report.initial_state = x0.';
report.state_feedback_gains = stateFeedbackGains;
report.output_feedback_gains = outputFeedbackGains;
report.responses = struct();

    function states = simulateResponse(systemMatrix, initialState, timeGrid)
        states = zeros(length(timeGrid), length(initialState));
        for localIdx = 1:length(timeGrid)
            states(localIdx, :) = (expm(systemMatrix * timeGrid(localIdx)) * initialState).';
        end
    end

    function saveResponsePlot(timeGrid, states, plotTitle, destination)
        fig = figure('Visible', 'off', 'Color', 'w');
        plot(timeGrid, states(:, 1), 'LineWidth', 2.4, 'Color', [0.04, 0.37, 1.0]);
        hold on;
        plot(timeGrid, states(:, 2), 'LineWidth', 2.4, 'Color', [0.82, 0.29, 0.36], 'LineStyle', '--');
        grid on;
        xlabel('t');
        ylabel('state');
        title(plotTitle);
        legend({'x_1(t)', 'x_2(t)'}, 'Location', 'northeast');
        exportgraphics(fig, destination, 'Resolution', 160);
        close(fig);
    end

openLoopStates = simulateResponse(A, x0, tGrid);
saveResponsePlot(tGrid, openLoopStates, 'Open-Loop State Response', fullfile(figureDir, figureNames.open_loop));
report.responses.open_loop = struct( ...
    'closed_loop_matrix', A, ...
    'final_state', openLoopStates(end, :), ...
    'eigenvalues', [real(eig(A)), imag(eig(A))] ...
);

stateKeys = fieldnames(stateFeedbackGains);
for idx = 1:numel(stateKeys)
    key = stateKeys{idx};
    K = stateFeedbackGains.(key);
    Acl = A + B * K;
    states = simulateResponse(Acl, x0, tGrid);
    saveResponsePlot(tGrid, states, ['State-Feedback Response (' strrep(key, '_', ' ') ')'], fullfile(figureDir, figureNames.(key)));
    report.responses.(key) = struct( ...
        'closed_loop_matrix', Acl, ...
        'gain', K, ...
        'final_state', states(end, :), ...
        'eigenvalues', [real(eig(Acl)), imag(eig(Acl))] ...
    );
end

outputKeys = fieldnames(outputFeedbackGains);
for idx = 1:numel(outputKeys)
    key = outputKeys{idx};
    K = outputFeedbackGains.(key);
    Acl = A + B * K * C;
    states = simulateResponse(Acl, x0, tGrid);
    saveResponsePlot(tGrid, states, ['Output-Feedback Response (' strrep(key, '_', ' ') ')'], fullfile(figureDir, figureNames.(key)));
    report.responses.(key) = struct( ...
        'closed_loop_matrix', Acl, ...
        'gain', K, ...
        'final_state', states(end, :), ...
        'eigenvalues', [real(eig(Acl)), imag(eig(Acl))] ...
    );
end

reportPath = fullfile(generatedDir, 'control_report.json');
fid = fopen(reportPath, 'w');
fprintf(fid, '%s', jsonencode(report, PrettyPrint=true));
fclose(fid);

fprintf('Saved MATLAB report to: %s\n', generatedDir);
fprintf('Saved MATLAB figures to: %s\n', figureDir);
