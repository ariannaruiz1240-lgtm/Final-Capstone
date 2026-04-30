% =========================================================================
% MULTI-BAND GENERATOR — INTERACTIVE GPS SETUP
% 1. Type a rough center point to fetch the initial satellite image
% 2. Click SW corner, then NE corner to define the grid
% 3. Click START, then END on the image
% 4. Click TOWERS on the image, type heights
% 5. Click OBSTACLE corners on the image, type heights
% 6. All GPS coords converted to meters to build the SINR matrix
% 7. Saves C_Band_L.mat compatible with the A* planner
% =========================================================================
clear; clc; close all;

%% --- 0. BAND SELECTION ---
% 1 = 700 MHz (Low Band)
% 2 = 2.1 GHz (Mid Band)
% 3 = 3.5 GHz (C-Band)
ACTIVE_BAND = 3;

%% --- 1. BAND DEFINITIONS ---
Bands(1).Name      = '700 MHz (Low Band)';
Bands(1).fc        = 700e6;
Bands(1).Wall_Loss = 8;

Bands(2).Name      = '2.1 GHz (Mid Band)';
Bands(2).fc        = 2.1e9;
Bands(2).Wall_Loss = 20;

Bands(3).Name      = '3.5 GHz (C-Band)';
Bands(3).fc        = 3.5e9;
Bands(3).Wall_Loss = 40;

fc        = double(Bands(ACTIVE_BAND).fc);
Wall_Loss = Bands(ACTIVE_BAND).Wall_Loss;
fprintf('Band: %s\n', Bands(ACTIVE_BAND).Name);

%% --- 2. INITIAL INPUTS ---
fprintf('\n--- Initial Setup ---\n');
fprintf('Enter a rough center coordinate to load the satellite image.\n\n');

%Latitude: 38.978647
%Longitude: -76.452925

center_lat_in = input('Approximate center Latitude  (deg): ');
center_lon_in = input('Approximate center Longitude (deg): ');

res_xy = input('\nXY voxel resolution (m) [default 2]: ');
if isempty(res_xy), res_xy = 2; end
res_z  = input('Z  voxel resolution (m) [default 2]: ');
if isempty(res_z),  res_z  = 2; end

z_min_m = input('Min altitude AGL (m) [default 0]: ');
if isempty(z_min_m), z_min_m = 0; end
z_max_m = input('Max altitude AGL (m) [default 200]: ');
if isempty(z_max_m), z_max_m = 200; end

%% --- 3. FETCH SATELLITE IMAGE ---
fprintf('\nFetching satellite image...\n');

zoom    = 16;
n       = 2^zoom;
lat_rad = deg2rad(center_lat_in);
tile_x  = floor((center_lon_in + 180) / 360 * n);
tile_y  = floor((1 - log(tan(lat_rad) + sec(lat_rad)) / pi) / 2 * n);

num_tiles_x = 3;
num_tiles_y = 3;
tx_start = tile_x - floor(num_tiles_x/2);
ty_start = tile_y - floor(num_tiles_y/2);

tile_imgs = cell(num_tiles_y, num_tiles_x);
for row = 1:num_tiles_y
    for col = 1:num_tiles_x
        tx = tx_start + (col - 1);
        ty = ty_start + (row - 1);
        tile_url = sprintf( ...
            'https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/%d/%d/%d', ...
            zoom, ty, tx);
        fname = sprintf('tile_%d_%d.png', row, col);
        try
            websave(fname, tile_url);
            tile_imgs{row, col} = imread(fname);
        catch
            warning('Failed to fetch tile (%d,%d) — using grey fallback.', row, col);
            tile_imgs{row, col} = uint8(50 * ones(256, 256, 3));
        end
    end
end

sat_img = cell2mat(cellfun(@(r) cat(2, r{:}), ...
    num2cell(tile_imgs, 2), 'UniformOutput', false));

img_lon_W = tx_start / n * 360 - 180;
img_lat_N = rad2deg(atan(sinh(pi * (1 - 2 * ty_start / n))));
img_lon_E = (tx_start + num_tiles_x) / n * 360 - 180;
img_lat_S = rad2deg(atan(sinh(pi * (1 - 2 * (ty_start + num_tiles_y) / n))));

%% --- 4. DISPLAY MAP ---
fig = figure('Color', 'k', 'Position', [100 80 1100 800]);
image([img_lon_W, img_lon_E], [img_lat_S, img_lat_N], sat_img);
set(gca, 'YDir', 'normal', 'Color', 'k', 'XColor', 'w', 'YColor', 'w');
hold on;
xlim([img_lon_W, img_lon_E]);
ylim([img_lat_S, img_lat_N]);
daspect([1/cosd(center_lat_in), 1, 1]);
xlabel('Longitude (°)'); ylabel('Latitude (°)');
grid on; set(gca, 'GridColor', 'w', 'GridAlpha', 0.15);

%% --- 5. CLICK GRID CORNERS ---
title('Click the SW (bottom-left) corner of your grid', 'Color', 'c', 'FontSize', 14);
[lon_SW, lat_SW] = ginput(1);
plot(lon_SW, lat_SW, 'cs', 'MarkerFaceColor', 'c', 'MarkerSize', 12, ...
    'DisplayName', 'SW Corner');
fprintf('SW corner: %.6f lat, %.6f lon\n', lat_SW, lon_SW);

title('Click the NE (top-right) corner of your grid', 'Color', 'c', 'FontSize', 14);
[lon_NE, lat_NE] = ginput(1);
plot(lon_NE, lat_NE, 'cs', 'MarkerFaceColor', 'c', 'MarkerSize', 12, ...
    'DisplayName', 'NE Corner');
fprintf('NE corner: %.6f lat, %.6f lon\n', lat_NE, lon_NE);

grid_lons = [lon_SW, lon_NE, lon_NE, lon_SW, lon_SW];
grid_lats = [lat_SW, lat_SW, lat_NE, lat_NE, lat_SW];
plot(grid_lons, grid_lats, '--c', 'LineWidth', 1.5, 'DisplayName', 'Grid boundary');
drawnow;

ref_lat = lat_SW;
ref_lon = lon_SW;

m_per_deg_lat = 111320;
m_per_deg_lon = 111320 * cosd(ref_lat);

gps2x = @(lat) (lat - ref_lat) * m_per_deg_lat;
gps2y = @(lon) (lon - ref_lon) * m_per_deg_lon;

%% --- 6. CLICK START & GOAL ---
title('Click the START point', 'Color', 'g', 'FontSize', 14);
[start_lon_click, start_lat_click] = ginput(1);
plot(start_lon_click, start_lat_click, 'gs', ...
    'MarkerFaceColor', 'g', 'MarkerSize', 12, 'DisplayName', 'Start');
fprintf('Start: %.6f lat, %.6f lon\n', start_lat_click, start_lon_click);

title('Click the GOAL point', 'Color', 'r', 'FontSize', 14);
[goal_lon_click, goal_lat_click] = ginput(1);
plot(goal_lon_click, goal_lat_click, 'rs', ...
    'MarkerFaceColor', 'r', 'MarkerSize', 12, 'DisplayName', 'Goal');
fprintf('Goal : %.6f lat, %.6f lon\n', goal_lat_click, goal_lon_click);

start_alt_m = input('\nStart altitude AGL (m): ');
goal_alt_m  = input('Goal  altitude AGL (m): ');

%% --- 7. CLICK TOWERS ---
num_towers  = input('\nHow many cell towers? ');
tower_lat   = zeros(num_towers, 1);
tower_lon   = zeros(num_towers, 1);
tower_alt_m = zeros(num_towers, 1);

for t = 1:num_towers
    title(sprintf('Click TOWER %d of %d', t, num_towers), 'Color', 'y', 'FontSize', 14);
    [tw_lon, tw_lat] = ginput(1);
    tower_lat(t) = tw_lat;
    tower_lon(t) = tw_lon;
    plot(tw_lon, tw_lat, '^y', 'MarkerFaceColor', 'y', 'MarkerSize', 10, ...
        'DisplayName', sprintf('Tower %d', t));
    text(tw_lon + 0.00005, tw_lat + 0.00005, sprintf('T%d', t), ...
        'Color', 'y', 'FontSize', 9);
    fprintf('Tower %d: %.6f lat, %.6f lon\n', t, tw_lat, tw_lon);
    tower_alt_m(t) = input(sprintf('Tower %d height AGL (m): ', t));
end

%% --- 8. CLICK OBSTACLES ---
num_obstacles = input('\nHow many obstacles? (0 for none): ');
Obstacles    = zeros(num_obstacles, 6);
ObstacleList = struct([]);

if num_obstacles > 0
    fprintf('\nFor each obstacle: click SW corner then NE corner, then type height.\n\n');

    for o = 1:num_obstacles
        title(sprintf('Obstacle %d of %d — click SW corner', o, num_obstacles), ...
            'Color', [1 0.5 0], 'FontSize', 14);
        [obs_lon_SW, obs_lat_SW] = ginput(1);
        plot(obs_lon_SW, obs_lat_SW, 's', 'Color', [1 0.5 0], ...
            'MarkerFaceColor', [1 0.5 0], 'MarkerSize', 8);
        fprintf('  Obstacle %d SW: %.6f lat, %.6f lon\n', o, obs_lat_SW, obs_lon_SW);

        title(sprintf('Obstacle %d of %d — click NE corner', o, num_obstacles), ...
            'Color', [1 0.5 0], 'FontSize', 14);
        [obs_lon_NE, obs_lat_NE] = ginput(1);
        plot(obs_lon_NE, obs_lat_NE, 's', 'Color', [1 0.5 0], ...
            'MarkerFaceColor', [1 0.5 0], 'MarkerSize', 8);
        fprintf('  Obstacle %d NE: %.6f lat, %.6f lon\n', o, obs_lat_NE, obs_lon_NE);

        % Draw filled rectangle
        obs_rect_lon = [obs_lon_SW, obs_lon_NE, obs_lon_NE, obs_lon_SW, obs_lon_SW];
        obs_rect_lat = [obs_lat_SW, obs_lat_SW, obs_lat_NE, obs_lat_NE, obs_lat_SW];
        fill(obs_rect_lon, obs_rect_lat, [1 0.5 0], ...
            'FaceAlpha', 0.35, 'EdgeColor', [1 0.5 0], 'LineWidth', 1.5, ...
            'DisplayName', sprintf('Obstacle %d', o));
        text(mean([obs_lon_SW, obs_lon_NE]), mean([obs_lat_SW, obs_lat_NE]), ...
            sprintf('OBS %d', o), 'Color', 'w', 'FontSize', 9, ...
            'HorizontalAlignment', 'center');
        drawnow;

        obs_height_m = input(sprintf('  Obstacle %d height (m): ', o));

        % Convert to meters
        obs_xmin = gps2x(obs_lon_SW);
        obs_xmax = gps2x(obs_lon_NE);
        obs_ymin = gps2y(obs_lat_SW);
        obs_ymax = gps2y(obs_lat_NE);

        Obstacles(o, :) = [ min(obs_xmin, obs_xmax), max(obs_xmin, obs_xmax), ...
                            min(obs_ymin, obs_ymax), max(obs_ymin, obs_ymax), ...
                            0, obs_height_m ];

        % Build 4 corners in meters for GPS back-conversion
        x1 = Obstacles(o,1); x2 = Obstacles(o,2);
        y1 = Obstacles(o,3); y2 = Obstacles(o,4);
        corners_m = [x1 y1; x2 y1; x2 y2; x1 y2];

        % Store struct with GPS coordinates
        ObstacleList(o).corners_m   = corners_m;
        ObstacleList(o).corners_lat = ref_lat + corners_m(:,1) / m_per_deg_lat;
        ObstacleList(o).corners_lon = ref_lon + corners_m(:,2) / m_per_deg_lon;
        ObstacleList(o).height      = obs_height_m;
        ObstacleList(o).aabb        = [Obstacles(o,1), Obstacles(o,2), ...
                                       Obstacles(o,3), Obstacles(o,4)];

        fprintf('  Obstacle %d: X[%.1f %.1f] Y[%.1f %.1f] Z[0 %.1f] m\n', ...
            o, Obstacles(o,1), Obstacles(o,2), Obstacles(o,3), Obstacles(o,4), obs_height_m);
    end
end

title('Setup complete — generating SINR matrix...', 'Color', 'w', 'FontSize', 13);
legend('Location', 'northwest', 'TextColor', 'w', 'Color', 'k');
drawnow;

%% --- 9. DERIVE GRID FROM CLICKED CORNERS ---
x_max_m = gps2x(lat_NE);
y_max_m = gps2y(lon_NE);

x_range = 0 : res_xy : x_max_m;
y_range = 0 : res_xy : y_max_m;
z_range = z_min_m : res_z : z_max_m;

fprintf('\nGrid size : %.0f m x %.0f m x %.0f m\n', x_max_m, y_max_m, z_max_m - z_min_m);
fprintf('Voxels    : %d x %d x %d = %d\n', ...
    length(x_range), length(y_range), length(z_range), ...
    length(x_range) * length(y_range) * length(z_range));

%% --- 10. CONVERT ALL GPS → METERS ---
start_x_m = gps2x(start_lat_click);
start_y_m = gps2y(start_lon_click);
start_z_m = start_alt_m;

goal_x_m  = gps2x(goal_lat_click);
goal_y_m  = gps2y(goal_lon_click);
goal_z_m  = goal_alt_m;

BS_Pos = zeros(num_towers, 3);
for t = 1:num_towers
    BS_Pos(t,1) = gps2y(tower_lon(t));   % meshgrid row → A* X
    BS_Pos(t,2) = gps2x(tower_lat(t));   % meshgrid col → A* Y
    BS_Pos(t,3) = tower_alt_m(t);
end

fprintf('\n--- Converted Coordinates (meters from SW corner) ---\n');
fprintf('Start   : [%.1f, %.1f, %.1f]\n', start_x_m, start_y_m, start_z_m);
fprintf('Goal    : [%.1f, %.1f, %.1f]\n', goal_x_m,  goal_y_m,  goal_z_m);
for t = 1:num_towers
    fprintf('Tower %d : [%.1f, %.1f, %.1f]\n', t, BS_Pos(t,1), BS_Pos(t,2), BS_Pos(t,3));
end

%% --- 11. BUILD MESHGRID ---
[X, Y, Z] = meshgrid(x_range, y_range, z_range);
[rows, cols, heights] = size(X);

%% --- 12. PHYSICS ENGINE ---
Pt_dBm         = 50;
Noise_dBm      = -100;
N_elements     = 16;
Tilt_mech_deg  = 6;
Max_Array_Gain = 8 + 10*log10(N_elements);

c         = 3e8;
lambda    = c / fc;
d_spacing = 0.5 * lambda;
beta      = -2 * pi * (d_spacing/lambda) * sin(deg2rad(-Tilt_mech_deg));
Noise_mW  = 10^(Noise_dBm / 10);

Rx_Power_mW = zeros(rows, cols, heights, num_towers);

for t = 1:num_towers
    fprintf('Computing tower %d of %d...\n', t, num_towers);
    for k = 1:heights
        z_current = z_range(k);

        X_sl = X(:,:,k);
        Y_sl = Y(:,:,k);

        dx = X_sl - BS_Pos(t,1);
        dy = Y_sl - BS_Pos(t,2);
        dz = z_current - BS_Pos(t,3);

        dist = sqrt(dx.^2 + dy.^2 + dz.^2);
        dist(dist < 1) = 1;

        Theta_rad = atan2(dz, sqrt(dx.^2 + dy.^2));
        psi    = 2 * pi * (d_spacing/lambda) * sin(Theta_rad) + beta;
        AF_Val = sin(N_elements * psi / 2) ./ (N_elements * sin(psi / 2) + 1e-9);

        Total_Gain = Max_Array_Gain ...
                   + 20*log10(abs(AF_Val)) ...
                   + 10*log10(cos(Theta_rad).^2 + 0.001);

        Shadow_Map_dB = zeros(rows, cols);
        for obs = 1:size(Obstacles, 1)
            bx = Obstacles(obs, 1:2);
            by = Obstacles(obs, 3:4);
            bz = Obstacles(obs, 5:6);
            if z_current > bz(2), continue; end

            b_center  = [(bx(1)+bx(2))/2, (by(1)+by(2))/2];
            vec_to_b  = b_center - BS_Pos(t,1:2);
            dist_to_b = norm(vec_to_b);

            vec_px    = X_sl - BS_Pos(t,1);
            vec_py    = Y_sl - BS_Pos(t,2);
            dist_to_p = sqrt(vec_px.^2 + vec_py.^2) + 1e-9;

            dot_prod  = (vec_px .* vec_to_b(1) + vec_py .* vec_to_b(2)) ...
                        ./ (dist_to_p .* dist_to_b);
            dot_prod  = max(-1, min(1, dot_prod));
            angle_diff    = acos(dot_prod);
            b_width       = max(bx(2)-bx(1), by(2)-by(1));
            angular_width = (b_width/2) / dist_to_b;

            in_shadow = (dist_to_p > dist_to_b) & (abs(angle_diff) < angular_width);
            Shadow_Map_dB(in_shadow) = max(Shadow_Map_dB(in_shadow), Wall_Loss);
        end

        PL     = 20*log10(dist) + 20*log10(fc/1e9) + 32.44;
        Rx_dBm = Pt_dBm + Total_Gain - (PL + Shadow_Map_dB);
        Rx_Power_mW(:,:,k,t) = 10.^(Rx_dBm / 10);
    end
end

%% --- 13. SINR ---
fprintf('Computing SINR...\n');
Total_Power_mW = sum(Rx_Power_mW, 4);
SNR_Matrix     = -200 * ones(rows, cols, heights);
serving_tower  = zeros(rows, cols, heights, 'uint8');

for t = 1:num_towers
    Signal       = Rx_Power_mW(:,:,:,t);
    Interference = Total_Power_mW - Signal;
    SINR_dB      = 10*log10(Signal ./ (Interference + Noise_mW));
    better = SINR_dB > SNR_Matrix;
    SNR_Matrix(better)    = SINR_dB(better);
    serving_tower(better) = t;
end

fprintf('SINR range: %.1f dB to %.1f dB\n', min(SNR_Matrix(:)), max(SNR_Matrix(:)));

%% --- 14. OBSTACLE HARD NULL ---
obs_mask = false(rows, cols, heights);
for obs = 1:size(Obstacles, 1)
    obs_mask = obs_mask | ...
        (X >= Obstacles(obs,1) & X <= Obstacles(obs,2) & ...
         Y >= Obstacles(obs,3) & Y <= Obstacles(obs,4) & ...
         Z >= Obstacles(obs,5) & Z <= Obstacles(obs,6));
end
SNR_Matrix(obs_mask) = -120;

%% --- 15. BOUNDARY BUFFERS ---
buffer = 5;
Boundaries = zeros(0, 6);
if ~isempty(Obstacles) && size(Obstacles,1) > 0
    Boundaries = Obstacles;
    Boundaries(:,1) = Obstacles(:,1) - buffer;
    Boundaries(:,2) = Obstacles(:,2) + buffer;
    Boundaries(:,3) = Obstacles(:,3) - buffer;
    Boundaries(:,4) = Obstacles(:,4) + buffer;
end

%% --- 16. START/GOAL → VOXEL INDICES ---
start_vox = [ round(start_y_m / res_xy) + 1, ...
              round(start_x_m / res_xy) + 1, ...
              round((start_z_m - z_min_m) / res_z) + 1 ];

goal_vox  = [ round(goal_y_m  / res_xy) + 1, ...
              round(goal_x_m  / res_xy) + 1, ...
              round((goal_z_m  - z_min_m) / res_z) + 1 ];

nx = size(SNR_Matrix, 1);
ny = size(SNR_Matrix, 2);
nz = size(SNR_Matrix, 3);

start_vox = max(1, min([nx, ny, nz], start_vox));
goal_vox  = max(1, min([nx, ny, nz], goal_vox));

fprintf('\n--- Voxel Indices ---\n');
fprintf('Start voxel : [%d, %d, %d]\n', start_vox);
fprintf('Goal  voxel : [%d, %d, %d]\n', goal_vox);

%% --- 17. SAVE ---
save('C_Band_Paper_Ex.mat', ...
    'SNR_Matrix',       'serving_tower',                    ...
    'x_range',          'y_range',          'z_range',      ...
    'Obstacles',        'Boundaries',       'BS_Pos',       ...
    'ObstacleList',                                         ...
    'ref_lat',          'ref_lon',          'res_xy',       'res_z', ...
    'start_vox',        'goal_vox',                         ...
    'start_lat_click',  'start_lon_click',  'start_alt_m',  ...
    'goal_lat_click',   'goal_lon_click',   'goal_alt_m',   ...
    'tower_lat',        'tower_lon',        'tower_alt_m',  ...
    'lat_SW',           'lon_SW',           'lat_NE',       'lon_NE', ...
    '-v7');

fprintf('\nSaved C_Band_L.mat  [%s | %d towers | %d obstacles | %d voxels]\n', ...
    Bands(ACTIVE_BAND).Name, num_towers, num_obstacles, numel(SNR_Matrix));
fprintf('Start voxel : [%d, %d, %d]\n', start_vox);
fprintf('Goal  voxel : [%d, %d, %d]\n', goal_vox);