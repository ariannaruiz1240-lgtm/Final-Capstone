clear all; clc; clf;

%% --- Load Saved Waypoints ---
if ~exist('path_waypoints_paper_ex.mat', 'file')
    error('path_waypoints_paper_ex.mat not found. Run the A* planner first.');
end
load('path_waypoints_paper_ex.mat');

tmp = load('C_Band_Paper_Ex.mat', 'tower_lat', 'tower_lon', 'Obstacles');
tower_lat = tmp.tower_lat;
tower_lon = tmp.tower_lon;
Obstacles = tmp.Obstacles;
%%
% Load obstacle data from generator file
if exist('C_Band_Paper_Ex.mat', 'file')
    tmp = load('C_Band_Paper_Ex.mat', 'ObstacleList', 'Obstacles');
    if isfield(tmp, 'ObstacleList') && ~isempty(tmp.ObstacleList)
        ObstacleList  = tmp.ObstacleList;
        has_obstacles = true;
    else
        has_obstacles = false;
    end
else
    warning('C_Band_L.mat not found — obstacles will not be drawn.');
    has_obstacles = false;
end

%% --- GPS Bounding Box ---
m_per_deg_lat = 111320;
m_per_deg_lon = 111320 * cosd(ref_lat);

% ymax = lon direction, xmax = lat direction (swapped to match meshgrid convention)
grid_lat_min = ref_lat;
grid_lat_max = ref_lat + ((ymax - 1) * res_xy) / m_per_deg_lat;
grid_lon_min = ref_lon;
grid_lon_max = ref_lon + ((xmax - 1) * res_xy) / m_per_deg_lon;

center_lat = (grid_lat_min + grid_lat_max) / 2;
center_lon = (grid_lon_min + grid_lon_max) / 2;

%% --- Fetch Satellite Tiles ---
zoom    = 16;
n       = 2^zoom;
lat_rad = deg2rad(center_lat);
tile_x  = floor((center_lon + 180) / 360 * n);
tile_y  = floor((1 - log(tan(lat_rad) + sec(lat_rad)) / pi) / 2 * n);

num_tiles_x = 3;
num_tiles_y = 3;
tx_start = tile_x - floor(num_tiles_x/2);
ty_start = tile_y - floor(num_tiles_y/2);

tile_imgs = cell(num_tiles_y, num_tiles_x);
fprintf('Fetching satellite tiles...\n');
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
            warning('Failed to fetch tile (%d,%d).', row, col);
            tile_imgs{row, col} = uint8(50 * ones(256, 256, 3));
        end
    end
end

sat_img = cell2mat(cellfun(@(r) cat(2, r{:}), ...
    num2cell(tile_imgs, 2), 'UniformOutput', false));

%% --- GPS Bounds of Stitched Image ---
img_lon_W = tx_start / n * 360 - 180;
img_lat_N = rad2deg(atan(sinh(pi * (1 - 2 * ty_start / n))));
img_lon_E = (tx_start + num_tiles_x) / n * 360 - 180;
img_lat_S = rad2deg(atan(sinh(pi * (1 - 2 * (ty_start + num_tiles_y) / n))));

fprintf('Tile coverage: %.5f to %.5f lat, %.5f to %.5f lon\n', ...
    img_lat_S, img_lat_N, img_lon_W, img_lon_E);

%% --- Plot ---
figure('Color', 'k', 'Position', [1100 100 900 700]);
hold on;

image([img_lon_W, img_lon_E], [img_lat_S, img_lat_N], sat_img);
set(gca, 'YDir', 'normal');

% Path colored by altitude
scatter(path_lon, path_lat, 20, path_alt, 'filled', 'HandleVisibility', 'off');
cb2 = colorbar; ylabel(cb2, 'Altitude (m AGL)'); cb2.Color = 'w';
colormap(gca, jet);

% Path line
plot(path_lon, path_lat, '-w', 'LineWidth', 3, 'HandleVisibility', 'off');
plot(path_lon, path_lat, '-m', 'LineWidth', 1.5, 'DisplayName', 'Path');

% Start / goal markers
plot(start_lon, start_lat, 'gs', 'MarkerFaceColor', 'g', 'MarkerSize', 12, 'DisplayName', 'Start');
plot(goal_lon,  goal_lat,  'rs', 'MarkerFaceColor', 'r', 'MarkerSize', 12, 'DisplayName', 'Goal');

% Tower markers
for t = 1:length(tower_lat)
    plot(tower_lon(t), tower_lat(t), '^y', 'MarkerFaceColor', 'y', 'MarkerSize', 10, ...
        'DisplayName', sprintf('Tower %d', t));
    text(tower_lon(t) + 0.00003, tower_lat(t) + 0.00003, sprintf('T%d', t), ...
        'Color', 'y', 'FontSize', 9);
end

% Obstacle overlays
if has_obstacles
    for o = 1:length(ObstacleList)
        corners_lon = ObstacleList(o).corners_lon;
        corners_lat = ObstacleList(o).corners_lat;
        patch_lons  = [corners_lon; corners_lon(1)];
        patch_lats  = [corners_lat; corners_lat(1)];
        fill(patch_lats, patch_lons, [1 0.5 0], ...
            'FaceAlpha', 0.35, 'EdgeColor', [1 0.5 0], 'LineWidth', 1.5, ...
            'DisplayName', sprintf('Obstacle %d', o));
        text(mean(corners_lon), mean(corners_lat), sprintf('OBS%d', o), ...
            'Color', 'w', 'FontSize', 9, 'HorizontalAlignment', 'center');
    end
end

% Axis limits
margin_lat = (grid_lat_max - grid_lat_min) * 0.05;
margin_lon = (grid_lon_max - grid_lon_min) * 0.05;
xlim([grid_lon_min - margin_lon, grid_lon_max + margin_lon]);
ylim([grid_lat_min - margin_lat, grid_lat_max + margin_lat]);
daspect([1/cosd(ref_lat), 1, 1]);

xlabel('Longitude (°)'); ylabel('Latitude (°)');
set(gca, 'Color', 'k', 'XColor', 'w', 'YColor', 'w');
legend('Location', 'southeast', 'TextColor', 'w', 'Color', 'k');
title(sprintf('GPS Path — %.5f°N  %.5f°W , w=1/a=1', ref_lat, abs(ref_lon)), 'Color', 'w');
grid on; set(gca, 'GridColor', 'w', 'GridAlpha', 0.15);