%clear all
%close all
clc

map=uint8(map_occupancy*255);
%map = mapdisp;
% Initialize variables ----------------------------------------------------
%load MAP0; mapwidth=400;mapheight=400; maporgx=fix(mapwidth/2)+1;maporgy=fix(mapheight/2)+1;Rx=maporgx; Ry=maporgy; Tx= 200; Ty=20;
%load Quanser_MAP
% Method options: 
%method = 'VISIBILITY_GRAPH';
method = 'POTENTIAL_FIELD';
%method = 'ASTAR';
X = 1:mapwidth;
X = X - maporgx;
Y = 1:mapheight;
Y = Y - maporgy;
maptitle = 'Occupancy map'; 
n = length(obs);

% Plot occupancy map ------------------------------------------------------
%figure(1)
% plot(X, zeros(1, mapwidth)), 
hold on
% plot(zeros(1, mapwidth), Y)


% Find waypoints ----------------------------------------------------------
if isequal(method, 'POTENTIAL_FIELD')
    % Set parameter values for Potential Field method -------
    % Radial boundary of each potential field from potential center
    POT_TH = 3; % 3 pixels 
    % Use repulsive force (FRC_DIR = 1) or normal vector to the repulsive 
    % force (FRC_DIR = 2)
    FRC_DIR = 2;
    % Velocity in pixels/decision cycle
    VEL = 2;
    [Wx Wy mapp] = potential_fields([Rx Ry], [Tx Ty], obs, ...
        POT_TH, FRC_DIR, VEL, map);
elseif isequal(method, 'VORONOI_DIAGRAM')
    [Vx, Vy, path] = find_voronoi_path(voronoi_diagram, [Rx Ry], ...
        [Tx Ty], map);
    Wx = path(:, 1);
    Wy = path(:, 2);

elseif isequal(method, 'ASTAR')
    MAP= map_occupancy;
    [Wx Wy]= A_Star([Rx Ry], [Tx Ty], MAP);
else
    [Wx Wy mapp] = visibility_graph([Rx Ry], [Tx Ty], obs, map);
end


% Plot robot's trajectory ------------------------------------------------- 
figure(2)
if ~isempty(Wx)
    wx = Wx - maporgx;
    wy = -(Wy - maporgy);
    plot([rx;wx], [ry;wy], 'LineWidth', 2, 'Color', [0.2 0.9 0.9])
end
%xlabel('X (cm)')
%ylabel('Y (cm)')
text(rx+5, ry, 'Robot');
text(tx+5, ty, 'Target');
grid off
hold off

py=wy;
px=wx;
