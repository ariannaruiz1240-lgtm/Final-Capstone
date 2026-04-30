clear all; clc;

%% --- 1. Load Data & Setup Grid ---
if ~exist('C_Band_Paper_Ex.mat', 'file')
    error('C_Band_Paper_Ex.mat not found.');
end
load('C_Band_Paper_Ex.mat');
[xmax, ymax, zmax] = size(SNR_Matrix);

%% --- 2. Start & Goal from Generator ---
if exist('start_vox', 'var') && exist('goal_vox', 'var')
    start = double(start_vox);
    goal  = double(goal_vox);
    fprintf('Loaded start/goal from C_Band_L.mat\n');
    fprintf('Start voxel : [%d, %d, %d]\n', start);
    fprintf('Goal  voxel : [%d, %d, %d]\n', goal);
else
    error('start_vox / goal_vox not found in C_Band_L.mat. Re-run the generator first.');
end

sX = start(1); sY = start(2); sZ = start(3);
gX = goal(1);  gY = goal(2);  gZ = goal(3);

if SNR_Matrix(sX, sY, sZ) <= -100
    warning('Start voxel is inside an obstacle (SNR = %.1f dB). Path may fail.', ...
        SNR_Matrix(sX, sY, sZ));
end
if SNR_Matrix(gX, gY, gZ) <= -100
    warning('Goal voxel is inside an obstacle (SNR = %.1f dB). Path may fail.', ...
        SNR_Matrix(gX, gY, gZ));
end

%% --- 3. Cost Map ---
alpha  = 1;
weight = 1;

SNR_Capped = min(SNR_Matrix, 25);
MAP = 25 - SNR_Matrix;

obs_indices = find(SNR_Matrix <= -100);
MAP(obs_indices) = inf;

%%SINR_floor_dB = 13;

%%low_sinr_indices = find(SNR_Matrix < SINR_floor_dB);
%MAP(low_sinr_indices) = inf;

%% --- 4. Heuristic ---
[X_grid, Y_grid, Z_grid] = meshgrid(1:ymax, 1:xmax, 1:zmax);

bound_mask = false(xmax, ymax, zmax);
for b = 1:size(Boundaries, 1)
    bound_mask = bound_mask | ...
        (X_grid >= Boundaries(b,1) & X_grid <= Boundaries(b,2) & ...
         Y_grid >= Boundaries(b,3) & Y_grid <= Boundaries(b,4) & ...
         Z_grid <= Boundaries(b,6));
end
MAP(bound_mask) = inf;

H = weight * sqrt((X_grid - gX).^2 + (Y_grid - gY).^2 + (Z_grid - gZ).^2);

%% --- 5. A* ---

% --- Simple Inflation ---
% This turns neighbors of obstacles into 'high cost' to prevent clipping
obs_mask = isinf(MAP);
inflated_obs = imdilate(obs_mask, ones(3,3,3)); % Expands obstacles by 1 voxel
MAP(inflated_obs & ~obs_mask) = 50; % Set a high penalty for being 'near' a wall
G_3D     = inf(xmax, ymax, zmax);
inOpen   = false(xmax, ymax, zmax);
inClosed = false(xmax, ymax, zmax);
openIdx  = zeros(xmax, ymax, zmax, 'uint32');

G_3D(sX, sY, sZ) = 0;

MAX_NODES   = xmax * ymax * zmax;
openNodes   = zeros(MAX_NODES, 6, 'single');
openCount   = 0;
closedNodes = zeros(MAX_NODES, 6, 'single');
closedCount = 0;

openCount = openCount + 1;
openNodes(openCount, :) = [sX, sY, sZ, 0, H(sX,sY,sZ), 0];
inOpen(sX, sY, sZ) = true;
openIdx(sX, sY, sZ) = openCount;

solved = false;

fprintf('Running A* search...\n');
tic;

%% --- 6. Main Search Loop ---
[dx, dy, dz] = ndgrid(-1:1, -1:1, -1:1);
dx = dx(:); dy = dy(:); dz = dz(:);
self = (dx==0 & dy==0 & dz==0);
dx(self) = []; dy(self) = []; dz(self) = [];
neighbor_dists = sqrt(dx.^2 + dy.^2 + dz.^2);

while openCount > 0
    
    [~, I] = min(openNodes(1:openCount, 5));
    current = openNodes(I, :);

    cx = current(1); cy = current(2); cz = current(3);
    inOpen(cx, cy, cz) = false;
    openIdx(cx, cy, cz) = 0;

    openNodes(I, :) = openNodes(openCount, :);
    mx = openNodes(I,1); my = openNodes(I,2); mz = openNodes(I,3);
    if inOpen(mx, my, mz)
        openIdx(mx, my, mz) = I;
    end
    openCount = openCount - 1;

    closedCount = closedCount + 1;
    closedNodes(closedCount, :) = current;
    currIdx = closedCount;
    inClosed(cx, cy, cz) = true;

    if norm(current(1:3) - goal) < 0.1
        solved = true;
        break;
    end

    for n = 1:length(dx)
        x = cx + dx(n);
        y = cy + dy(n);
        z = cz + dz(n);

        if x<1||x>xmax||y<1||y>ymax||z<1||z>zmax, continue; end
        if isinf(MAP(x,y,z)), continue; end
        if inClosed(x,y,z), continue; end

        step_cost = neighbor_dists(n) + (MAP(x,y,z) * alpha);
        step_cost = max(0.1, step_cost);
        newG = current(4) + step_cost;

        if inOpen(x,y,z)
            if newG < G_3D(x,y,z)
                G_3D(x,y,z) = newG;
                row = openIdx(x,y,z);
                openNodes(row, 4) = newG;
                openNodes(row, 5) = newG + H(x,y,z);
                openNodes(row, 6) = currIdx;
            end
        else
            G_3D(x,y,z) = newG;
            openCount = openCount + 1;
            openNodes(openCount, :) = [x, y, z, newG, newG + H(x,y,z), currIdx];
            inOpen(x,y,z) = true;
            openIdx(x,y,z) = openCount;
        end
    end
end

search_time = toc;
fprintf('A* search completed in %.2f seconds\n', search_time);
%% --- 7. Path Reconstruction & Save ---
if solved
    path = [];
    curr = closedNodes(closedCount, :);
    while curr(6) ~= 0
        path = [curr(1:3); path];
        curr = closedNodes(curr(6), :);
    end
    path = [start; path];

    % --- GPS Conversion (col 2 = lat direction, col 1 = lon direction) ---
    m_per_deg_lat = 111320;
    m_per_deg_lon = 111320 * cosd(ref_lat);

    path_lat = ref_lat + ((path(:,2) - 1) * res_xy) / m_per_deg_lat;
    path_lon = ref_lon + ((path(:,1) - 1) * res_xy) / m_per_deg_lon;
    path_alt = z_range(1) + (path(:,3) - 1) * res_z;

    start_lat = ref_lat + ((sY - 1) * res_xy) / m_per_deg_lat;
    start_lon = ref_lon + ((sX - 1) * res_xy) / m_per_deg_lon;
    start_alt = z_range(1) + (sZ - 1) * res_z;

    goal_lat  = ref_lat + ((gY - 1) * res_xy) / m_per_deg_lat;
    goal_lon  = ref_lon + ((gX - 1) * res_xy) / m_per_deg_lon;
    goal_alt  = z_range(1) + (gZ - 1) * res_z;

    % Tower GPS (BS_Pos col 1 = lon direction, col 2 = lat direction)
    tower_lat = ref_lat + (BS_Pos(:,1) * res_xy) / m_per_deg_lat;
    tower_lon = ref_lon + (BS_Pos(:,2) * res_xy) / m_per_deg_lon;
    tower_alt = BS_Pos(:,3);

    % --- Path Distance & SINR Summary ---
    path_sinr     = SNR_Matrix(sub2ind(size(SNR_Matrix), path(:,1), path(:,2), path(:,3)));
    segment_dists = sqrt(sum(diff(path).^2, 2));
    total_dist_m  = sum(segment_dists) * res_xy;
    average_SINR = mean(path_sinr)
    min_SINR = min(path_sinr)
    max_SINR = max(path_sinr)
    sum_SINR = sum(path_sinr)

    fprintf('\n--- Path Summary ---\n');
    fprintf('Search time        : %.2f s\n',   search_time);
    fprintf('Waypoints          : %d\n',      size(path,1));
    fprintf('Total distance     : %.2f m\n',  total_dist_m);
    fprintf('Total SINR         : %.2f dB\n', sum(path_sinr));
    fprintf('Average SINR       : %.2f dB\n', mean(path_sinr));
    fprintf('Min SINR           : %.2f dB\n', min(path_sinr));
    fprintf('Max SINR           : %.2f dB\n', max(path_sinr));

    % --- Save ---
    waypoints     = struct();
    waypoints.lat = path_lat;
    waypoints.lon = path_lon;
    waypoints.alt = path_alt;
    waypoints.n   = size(path, 1);

        % --- Convert ObstacleList corners from meters to GPS -

    save('path_waypoints_paper_ex.mat', ...
        'path_lat',   'path_lon',   'path_alt',   ...
        'start_lat',  'start_lon',  'start_alt',  ...
        'goal_lat',   'goal_lon',   'goal_alt',   ...
        'tower_lat',  'tower_lon',  'tower_alt',  ...
        'waypoints',  'ref_lat',    'ref_lon',    ...
        'xmax',       'ymax',       'res_xy',     ...
        'lat_SW',     'lon_SW',     'lat_NE',     'lon_NE', 'Obstacles',...
        "total_dist_m","average_SINR", "min_SINR",'max_SINR', "sum_SINR",'segment_dists', 'search_time');

    fprintf('Saved %d GPS waypoints to path_waypoints.mat\n', size(path,1));

else
    disp('No path found. Check that start/goal voxels are not inside obstacles.');
end