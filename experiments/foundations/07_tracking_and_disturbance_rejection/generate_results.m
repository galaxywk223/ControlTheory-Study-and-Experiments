repoRoot = fileparts(fileparts(fileparts(fileparts(mfilename('fullpath')))));
figureDir = fullfile(repoRoot, 'figures', '07_tracking_and_disturbance_rejection');
generatedDir = fullfile(repoRoot, 'generated', '07_tracking_and_disturbance_rejection');

if ~exist(figureDir, 'dir')
    mkdir(figureDir);
end
if ~exist(generatedDir, 'dir')
    mkdir(generatedDir);
end

A = [0.0, 1.0; 1.0, 0.2];
B = [0.0; 1.0];
C = [1.0, 0.0];
reference = 1.0;
disturbanceMagnitude = 0.35;
disturbanceStart = 4.0;
tGrid = linspace(0.0, 12.0, 1500);

Kplace = place(A, B, [-1.2, -1.9]);
Kstate = -Kplace;
referenceGain = -1.0 / (C * ((A + B * Kstate) \ B));

Aaug = [A, zeros(2, 1); -C, 0.0];
Baug = [B; 0.0];
Kaug = place(Aaug, Baug, [-1.2, -1.9, -2.6]);
Kx = -Kaug(1:2);
Ki = -Kaug(3);

[~, xState] = ode45(@(t, x) state_feedback_dynamics(A, B, Kstate, referenceGain, reference, disturbanceMagnitude, disturbanceStart, t, x), tGrid, [0.0; 0.0]);
xState = xState.';

[~, zIntegral] = ode45(@(t, z) integral_servo_dynamics(A, B, C, Kx, Ki, reference, disturbanceMagnitude, disturbanceStart, t, z), tGrid, [0.0; 0.0; 0.0]);
zIntegral = zIntegral.';
xIntegral = zIntegral(1:2, :);
xiIntegral = zIntegral(3, :);

yState = C * xState;
yIntegral = C * xIntegral;
uState = Kstate * xState + referenceGain * reference;
uIntegral = Kx * xIntegral + Ki * xiIntegral;

fig1 = figure('Visible', 'off', 'Color', 'w');
plot(tGrid, yState, 'LineWidth', 1.8, 'Color', [0.82, 0.29, 0.36]);
hold on;
plot(tGrid, yIntegral, 'LineWidth', 1.8, 'Color', [0.04, 0.37, 1.00]);
yline(reference, '--', 'Color', [0.12, 0.12, 0.12], 'LineWidth', 1.0);
xline(disturbanceStart, ':', 'Color', [0.42, 0.46, 0.50], 'LineWidth', 1.0);
xlabel('t (s)');
ylabel('y(t)');
title('Tracking response with constant input disturbance');
grid on;
legend({'State feedback', 'Integral servo', 'Reference', 'Disturbance starts'}, 'Location', 'southeast');
exportgraphics(fig1, fullfile(figureDir, 'tracking_response_under_disturbance.png'), 'Resolution', 160);
close(fig1);

fig2 = figure('Visible', 'off', 'Color', 'w');
plot(tGrid, uState, 'LineWidth', 1.8, 'Color', [0.82, 0.29, 0.36]);
hold on;
plot(tGrid, uIntegral, 'LineWidth', 1.8, 'Color', [0.04, 0.37, 1.00]);
xline(disturbanceStart, ':', 'Color', [0.42, 0.46, 0.50], 'LineWidth', 1.0);
xlabel('t (s)');
ylabel('u(t)');
title('Control input under reference tracking');
grid on;
legend({'State feedback', 'Integral servo'}, 'Location', 'northeast');
exportgraphics(fig2, fullfile(figureDir, 'tracking_control_input.png'), 'Resolution', 160);
close(fig2);

report.system_matrix_A = A;
report.input_matrix_B = B;
report.output_matrix_C = C;
report.reference = reference;
report.disturbance.magnitude = disturbanceMagnitude;
report.disturbance.start_time = disturbanceStart;
report.state_feedback.K = Kstate;
report.state_feedback.reference_gain = referenceGain;
report.state_feedback.steady_state_error = abs(yState(end) - reference);
report.state_feedback.overshoot = max(max(yState) - reference, 0.0);
report.state_feedback.control_peak_abs = max(abs(uState));
report.integral_servo.Kx = Kx;
report.integral_servo.Ki = Ki;
report.integral_servo.steady_state_error = abs(yIntegral(end) - reference);
report.integral_servo.overshoot = max(max(yIntegral) - reference, 0.0);
report.integral_servo.control_peak_abs = max(abs(uIntegral));

fid = fopen(fullfile(generatedDir, 'tracking_report_matlab.json'), 'w');
fprintf(fid, '%s', jsonencode(report, PrettyPrint=true));
fclose(fid);

fprintf('Saved MATLAB report to: %s\n', generatedDir);
fprintf('Saved MATLAB figures to: %s\n', figureDir);

function dx = state_feedback_dynamics(A, B, Kstate, referenceGain, reference, disturbanceMagnitude, disturbanceStart, t, x)
    if t >= disturbanceStart
        disturbance = disturbanceMagnitude;
    else
        disturbance = 0.0;
    end
    u = Kstate * x + referenceGain * reference;
    dx = A * x + B * (u + disturbance);
end

function dz = integral_servo_dynamics(A, B, C, Kx, Ki, reference, disturbanceMagnitude, disturbanceStart, t, z)
    x = z(1:2);
    xi = z(3);
    if t >= disturbanceStart
        disturbance = disturbanceMagnitude;
    else
        disturbance = 0.0;
    end
    u = Kx * x + Ki * xi;
    xDot = A * x + B * (u + disturbance);
    xiDot = reference - C * x;
    dz = [xDot; xiDot];
end
