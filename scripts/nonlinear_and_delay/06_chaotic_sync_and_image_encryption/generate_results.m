root = fileparts(fileparts(fileparts(fileparts(mfilename('fullpath')))));
fig_dir = fullfile(root, 'figures', '06_chaotic_sync_and_image_encryption');
gen_dir = fullfile(root, 'generated', '06_chaotic_sync_and_image_encryption');
source_image = fullfile(fig_dir, 'plaintext_input_image.jpg');

if ~exist(fig_dir, 'dir')
    mkdir(fig_dir);
end
if ~exist(gen_dir, 'dir')
    mkdir(gen_dir);
end

[time_driver, driver_only] = simulate_driver(80.0, 0.001, 1.0, [1.7; 2.5]);
[time_sync, drive, response, control] = simulate_sync(25.0, 0.001, 1.0);
error = response - drive;
seeds = derive_image_seeds(time_sync, drive);

image = imread(source_image);
[cipher, decrypted, row_order, col_order] = encrypt_image(image, seeds);
if ~isequal(image, decrypted)
    error('Decryption did not recover the original image.');
end

gray_original = rgb_to_gray(image);
gray_cipher = rgb_to_gray(cipher);

correlation_metrics = [
    adjacency_correlation(gray_original, 'horizontal');
    adjacency_correlation(gray_original, 'vertical');
    adjacency_correlation(gray_cipher, 'horizontal');
    adjacency_correlation(gray_cipher, 'vertical')
];

summary.sync_system.A = [-1.0, 0.0; 0.0, -1.0];
summary.sync_system.B = [2.0, -0.1; -5.0, 2.0];
summary.sync_system.B_delay = [-1.5, -0.1; -0.2, -1.5];
summary.sync_system.K = [32.8175, 47.7361; -133.1490, -79.0097];
summary.sync_system.tau = 1.0;
summary.sync_system.drive_history = [1.7; 2.5];
summary.sync_system.response_history = [1.0; 2.0];
summary.sync_metrics.settling_time_norm_le_1e_2 = settling_time(time_sync, error, 1.0e-2);
summary.sync_metrics.settling_time_norm_le_1e_4 = settling_time(time_sync, error, 1.0e-4);
summary.sync_metrics.max_error_norm = max(vecnorm(error, 2, 1));
summary.sync_metrics.max_control_abs = max(abs(control), [], 2);
summary.sync_metrics.final_error = error(:, end);
summary.image_encryption.source_image = 'figures/06_chaotic_sync_and_image_encryption/plaintext_input_image.jpg';
summary.image_encryption.seeds = seeds;
summary.image_encryption.image_shape = size(image);
summary.image_encryption.decryption_exact = true;
summary.image_encryption.correlation_labels = {'original_horizontal', 'original_vertical', 'encrypted_horizontal', 'encrypted_vertical'};
summary.image_encryption.correlation_values = correlation_metrics;
summary.image_encryption.row_perm_checksum = sum(double(row_order(1:20)));
summary.image_encryption.col_perm_checksum = sum(double(col_order(1:20)));

fid = fopen(fullfile(gen_dir, 'summary.json'), 'w');
fwrite(fid, jsonencode(summary, PrettyPrint=true), 'char');
fclose(fid);

fid = fopen(fullfile(gen_dir, 'correlation_metrics.csv'), 'w');
fprintf(fid, 'image,direction,value\n');
fprintf(fid, 'original,horizontal,%.12f\n', correlation_metrics(1));
fprintf(fid, 'original,vertical,%.12f\n', correlation_metrics(2));
fprintf(fid, 'encrypted,horizontal,%.12f\n', correlation_metrics(3));
fprintf(fid, 'encrypted,vertical,%.12f\n', correlation_metrics(4));
fclose(fid);

figure('Position', [120, 120, 620, 580], 'Color', 'w');
mask = time_driver >= 20.0;
plot(driver_only(1, mask), driver_only(2, mask), 'Color', [0.12, 0.35, 0.65], 'LineWidth', 0.9);
xlabel('x_1(t)');
ylabel('x_2(t)');
title('Drive-system phase portrait');
grid on;
exportgraphics(gcf, fullfile(fig_dir, 'drive_phase_portrait.png'), 'Resolution', 220);
close(gcf);

figure('Position', [140, 140, 940, 560], 'Color', 'w');
tiledlayout(2, 1, 'Padding', 'compact', 'TileSpacing', 'compact');

nexttile;
plot(time_sync, error(1, :), 'LineWidth', 1.8, 'DisplayName', 'e_1(t)');
hold on;
plot(time_sync, error(2, :), 'LineWidth', 1.8, 'DisplayName', 'e_2(t)');
ylabel('Error state');
title('Synchronization error trajectories');
legend('Location', 'northeast');
grid on;

nexttile;
semilogy(time_sync, max(vecnorm(error, 2, 1), 1.0e-12), 'Color', [0.74, 0.26, 0.17], 'LineWidth', 1.8);
xlabel('Time (s)');
ylabel('||e(t)||_2');
grid on;
exportgraphics(gcf, fullfile(fig_dir, 'synchronization_error.png'), 'Resolution', 220);
close(gcf);

figure('Position', [160, 160, 840, 760], 'Color', 'w');
tiledlayout(2, 2, 'Padding', 'compact', 'TileSpacing', 'compact');

nexttile;
imshow(image);
title('Original image');

nexttile;
imagesc(image(:, :, 1));
axis image off;
colormap(gca, 'hot');
title('Red channel');

nexttile;
imagesc(image(:, :, 2));
axis image off;
colormap(gca, 'summer');
title('Green channel');

nexttile;
imagesc(image(:, :, 3));
axis image off;
colormap(gca, 'winter');
title('Blue channel');
exportgraphics(gcf, fullfile(fig_dir, 'plaintext_rgb_channels.png'), 'Resolution', 220);
close(gcf);

figure('Position', [180, 180, 1080, 420], 'Color', 'w');
tiledlayout(1, 3, 'Padding', 'compact', 'TileSpacing', 'compact');
nexttile; imshow(image); title('Original');
nexttile; imshow(cipher); title('Encrypted');
nexttile; imshow(decrypted); title('Decrypted');
exportgraphics(gcf, fullfile(fig_dir, 'encryption_pipeline.png'), 'Resolution', 220);
close(gcf);

figure('Position', [200, 200, 1080, 620], 'Color', 'w');
tiledlayout(2, 3, 'Padding', 'compact', 'TileSpacing', 'compact');
channel_titles = {'Red', 'Green', 'Blue'};
for idx = 1:3
    nexttile;
    histogram(image(:, :, idx), 0:256, 'FaceColor', [0.85, 0.29 + 0.1 * idx, 0.35 + 0.15 * idx], 'EdgeColor', 'none');
    xlim([0, 255]);
    title(['Original ' channel_titles{idx}]);
    grid on;

    nexttile;
    histogram(cipher(:, :, idx), 0:256, 'FaceColor', [0.85, 0.29 + 0.1 * idx, 0.35 + 0.15 * idx], 'EdgeColor', 'none');
    xlim([0, 255]);
    title(['Encrypted ' channel_titles{idx}]);
    grid on;
end
exportgraphics(gcf, fullfile(fig_dir, 'rgb_histograms_before_after_encryption.png'), 'Resolution', 220);
close(gcf);

figure('Position', [220, 220, 920, 820], 'Color', 'w');
tiledlayout(2, 2, 'Padding', 'compact', 'TileSpacing', 'compact');

plot_correlation_scatter(nexttile, gray_original, 'horizontal', 'Original horizontal', [0.12, 0.35, 0.65]);
plot_correlation_scatter(nexttile, gray_original, 'vertical', 'Original vertical', [0.18, 0.62, 0.28]);
plot_correlation_scatter(nexttile, gray_cipher, 'horizontal', 'Encrypted horizontal', [0.74, 0.26, 0.17]);
plot_correlation_scatter(nexttile, gray_cipher, 'vertical', 'Encrypted vertical', [0.49, 0.36, 0.65]);
exportgraphics(gcf, fullfile(fig_dir, 'adjacent_pixel_correlation.png'), 'Resolution', 220);
close(gcf);

fprintf('Chaotic synchronization and image encryption reproduction completed.\n');
fprintf('Seeds: %.6f %.6f %.6f\n', seeds(1), seeds(2), seeds(3));
fprintf('Sync settling time ||e||<=1e-2: %.3fs\n', summary.sync_metrics.settling_time_norm_le_1e_2);
fprintf('Encrypted horizontal correlation: %.6f\n', correlation_metrics(3));

function [time, states] = simulate_driver(horizon, dt, tau, history)
    A = -eye(2);
    B = [2.0, -0.1; -5.0, 2.0];
    Bd = [-1.5, -0.1; -0.2, -1.5];
    delay_steps = round(tau / dt);
    total_steps = round(horizon / dt);
    time_full = linspace(-tau, horizon, total_steps + delay_steps + 1);
    states_full = zeros(2, length(time_full));
    states_full(:, 1:(delay_steps + 1)) = repmat(history, 1, delay_steps + 1);

    for k = (delay_steps + 1):(length(time_full) - 1)
        delayed_state = states_full(:, k - delay_steps);
        derivative = A * states_full(:, k) + B * tanh(states_full(:, k)) + Bd * tanh(delayed_state);
        states_full(:, k + 1) = states_full(:, k) + dt * derivative;
    end

    time = time_full((delay_steps + 1):end);
    states = states_full(:, (delay_steps + 1):end);
end

function [time, drive, response, control] = simulate_sync(horizon, dt, tau)
    A = -eye(2);
    B = [2.0, -0.1; -5.0, 2.0];
    Bd = [-1.5, -0.1; -0.2, -1.5];
    K = [32.8175, 47.7361; -133.1490, -79.0097];

    drive_history = [1.7; 2.5];
    response_history = [1.0; 2.0];

    delay_steps = round(tau / dt);
    total_steps = round(horizon / dt);
    time_full = linspace(-tau, horizon, total_steps + delay_steps + 1);
    drive_full = zeros(2, length(time_full));
    response_full = zeros(2, length(time_full));
    control_full = zeros(2, length(time_full));
    drive_full(:, 1:(delay_steps + 1)) = repmat(drive_history, 1, delay_steps + 1);
    response_full(:, 1:(delay_steps + 1)) = repmat(response_history, 1, delay_steps + 1);

    for k = (delay_steps + 1):(length(time_full) - 1)
        drive_delay = drive_full(:, k - delay_steps);
        response_delay = response_full(:, k - delay_steps);
        error = response_full(:, k) - drive_full(:, k);
        u_val = K * error;
        control_full(:, k) = u_val;

        drive_derivative = A * drive_full(:, k) + B * tanh(drive_full(:, k)) + Bd * tanh(drive_delay);
        response_derivative = A * response_full(:, k) + B * tanh(response_full(:, k)) + Bd * tanh(response_delay) + u_val;
        drive_full(:, k + 1) = drive_full(:, k) + dt * drive_derivative;
        response_full(:, k + 1) = response_full(:, k) + dt * response_derivative;
    end

    time = time_full((delay_steps + 1):end);
    drive = drive_full(:, (delay_steps + 1):end);
    response = response_full(:, (delay_steps + 1):end);
    control = control_full(:, (delay_steps + 1):end);
end

function value = settling_time(time, error, threshold)
    error_norm = vecnorm(error, 2, 1);
    suffix_max = flip(cummax(flip(error_norm)));
    idx = find(suffix_max <= threshold, 1, 'first');
    if isempty(idx)
        value = NaN;
    else
        value = time(idx);
    end
end

function output = chaotic_sequence(seed, length_value, burn_in)
    if nargin < 3
        burn_in = 1000;
    end
    value = min(max(seed, 1.0e-6), 1.0 - 1.0e-6);
    for i = 1:burn_in
        value = mod(3.99 * value * (1.0 - value) + 0.27 * sin(pi * value), 1.0);
    end
    output = zeros(length_value, 1);
    for i = 1:length_value
        value = mod(3.99 * value * (1.0 - value) + 0.27 * sin(pi * value), 1.0);
        output(i) = value;
    end
end

function seeds = derive_image_seeds(time, drive)
    targets = [12.0, 16.0, 20.0];
    samples = zeros(2, 3);
    for i = 1:3
        [~, idx] = min(abs(time - targets(i)));
        samples(:, i) = drive(:, idx);
    end
    seed_1 = mod(abs(samples(1, 1)) * 1.0e4 + abs(samples(2, 2)) * 1.0e3, 1.0);
    seed_2 = mod(abs(samples(1, 2)) * 1.0e4 + abs(samples(2, 3)) * 1.0e3 + 0.314159, 1.0);
    seed_3 = mod(abs(samples(1, 3) - samples(2, 1)) * 1.0e4 + 0.271828, 1.0);
    seeds = min(max([seed_1; seed_2; seed_3], 1.0e-6), 1.0 - 1.0e-6);
end

function [cipher, decrypted, row_order, col_order] = encrypt_image(image, seeds)
    [height, width, channels] = size(image);
    [~, row_order] = sort(chaotic_sequence(seeds(1), height), 'ascend');
    [~, col_order] = sort(chaotic_sequence(seeds(2), width), 'ascend');
    permuted = image(row_order, :, :);
    permuted = permuted(:, col_order, :);

    key_length = height * width * channels;
    key_stream = mod(floor(256 * chaotic_sequence(seeds(3), key_length)) + mod((0:(key_length - 1))', 256), 256);
    key_image = reshape(uint8(key_stream), [height, width, channels]);
    cipher = bitxor(permuted, key_image);
    recovered = bitxor(cipher, key_image);

    inverse_row = zeros(height, 1);
    inverse_col = zeros(width, 1);
    inverse_row(row_order) = 1:height;
    inverse_col(col_order) = 1:width;
    decrypted = recovered(inverse_row, :, :);
    decrypted = decrypted(:, inverse_col, :);
end

function gray = rgb_to_gray(image)
    image_d = double(image);
    gray = uint8(round(0.299 * image_d(:, :, 1) + 0.587 * image_d(:, :, 2) + 0.114 * image_d(:, :, 3)));
end

function value = adjacency_correlation(gray, mode)
    gray_d = double(gray);
    switch mode
        case 'horizontal'
            left = gray_d(:, 1:end-1);
            right = gray_d(:, 2:end);
        otherwise
            left = gray_d(1:end-1, :);
            right = gray_d(2:end, :);
    end
    left = left(:);
    right = right(:);
    corr_mat = corrcoef(left, right);
    value = corr_mat(1, 2);
end

function plot_correlation_scatter(tile_axis, gray, mode, plot_title, color_value)
    gray_d = double(gray);
    switch mode
        case 'horizontal'
            left = gray_d(:, 1:end-1);
            right = gray_d(:, 2:end);
        otherwise
            left = gray_d(1:end-1, :);
            right = gray_d(2:end, :);
    end
    left = left(:);
    right = right(:);
    sample_count = min(4000, length(left));
    indices = round(linspace(1, length(left), sample_count));
    scatter(tile_axis, left(indices), right(indices), 8, 'MarkerEdgeColor', color_value, 'MarkerFaceColor', color_value, 'MarkerFaceAlpha', 0.22, 'MarkerEdgeAlpha', 0.22);
    xlim(tile_axis, [0, 255]);
    ylim(tile_axis, [0, 255]);
    xlabel(tile_axis, 'Pixel i');
    ylabel(tile_axis, 'Pixel i+1');
    title(tile_axis, plot_title);
    grid(tile_axis, 'on');
end
