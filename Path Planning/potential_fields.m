function [X, Y, mapp, obs_pot, tar_pot] = potential_fields(robot, ...
    target, obs, th, mtd, vel, map)
[row col depth] = size(map); 
n = length(obs);
X = [];
Y = [];
mapp = map;
mapp = draw_box(robot(1), robot(2), 1, 1, [1 0 0], mapp);
mapp = draw_box(target(1), target(2), 1, 1, [0 0 1], mapp);
POT = zeros(row, col);
for i=1:n
    if ~isempty(obs{i})
        coords = obs{i};
        index = i/(n+1);
        tmp_map = map(:, :, 1);
        id = find(tmp_map == index);
        pot = find_potential_field(row, col, coords, th);
        pot(id) = 1;
        POT = POT + pot;
        POT = min(POT, 1);
    end
end

TH = sqrt(row^2 + col^2);
tar_pot = find_target_potential_field(row, col, target, TH);
obs_pot = POT;

% Find force vector in x and y directions
[tFx tFy] = gradient(tar_pot);
[oFx oFy] = gradient(obs_pot);
oFx = -oFx;
oFy = -oFy;

rx = robot(1);
ry = robot(2);
Rx = round(rx);
Ry = round(ry);




preRx = Rx;
preRy = Ry;



tF = complex(tFx(Ry, Rx), tFy(Ry, Rx));
oF = complex(oFx(Ry, Rx), oFy(Ry, Rx));
angTh = 3;
angTh = angTh*(pi/180);
preAng = angle(tF);
tar = complex(target(1), target(2));
rbt = complex(Rx, Ry);
dist = abs(tar-rbt);
dist_th = 4;

while dist>dist_th
    if abs(oF)~=0
        if mtd==1
            ang = angle(tF + oF);
        else
            if abs(check_angle(angle(oF)-angle(tF)))<= angTh
                ang = angle(tF);
            else
                oF1 = complex(0, 1)*oF;
                oF2 = complex(0, -1)*oF;
                ang1 = check_angle(angle(oF1)-preAng);
                ang2 = check_angle(angle(oF2)-preAng);
                if abs(ang1)<=abs(ang2)
                    ang = angle(oF1);
                else
                    ang = angle(oF2);
                end
            end
        end
    else
        ang = angle(tF);
    end

    rx = rx + vel*cos(ang);
    ry = ry + vel*sin(ang);
    Rx = round(rx);
    Ry = round(ry);
    
    if Rx<1 
      Rx=1;
    end

    if Ry<1 
      Ry=1;
    end
    
    [xline yline mapp] = draw_line([preRx preRy], [Rx Ry], [0.2 0.9 0.9], mapp);
    preRx = Rx;
    preRy = Ry;
    
    rbt = complex(rx, ry);
    dist = abs(tar-rbt);
    if abs(check_angle(ang-preAng))>= 178*(pi/180)
        return;
    end
    preAng = ang;
    
    % AH
    if Ry>size(tFx,1)
        Ry= size(tFx,1);
    end
    
     if Rx>size(tFx,2)
        Rx= size(tFx,2);
    end
    
    tF = complex(tFx(Ry, Rx), tFy(Ry, Rx));
    oF = complex(oFx(Ry, Rx), oFy(Ry, Rx));
    
    X = [X; xline];
    Y = [Y; yline];

%     figure(1),image(mapp);
%     pause(0.05)
    
end

return;
%--------------------------------------------------------------------------
function [pot] = find_potential_field(row, col, coords, th)
pot = zeros(row, col);
y = coords(1, 2);
for x=coords(1, 1):coords(2, 1)
    pot = add_pot(x, y, th, row, col, pot);
end
y = coords(4, 2);
for x=coords(4, 1):coords(3, 1)
    pot = add_pot(x, y, th, row, col, pot);
end
x = coords(1, 1);
for y=coords(3, 2):coords(1, 2)
    pot = add_pot(x, y, th, row, col, pot);
end
x = coords(2, 1);
for y=coords(3, 2):coords(1, 2)
    pot = add_pot(x, y, th, row, col, pot);
end


return;
%--------------------------------------------------------------------------
function [left right top bottom] = check_boundary(x, y, step, row, col)
left = x - round(step);
right = x + round(step);
top = y - round(step);
bottom = y + round(step);
if left < 1
    left = 1;
end
if right > col
    right = col;
end
if top < 1
    top = 1;
end
if bottom > row 
    bottom = row;
end
return;
%--------------------------------------------------------------------------
function [X Y] = gen_mesh(left, right, top, bottom, orgx, orgy)
xindex = left:right;
xindex = xindex - orgx;
yindex = top:bottom;
yindex = yindex - orgy;
[X Y] = meshgrid(xindex, yindex);
return;
%--------------------------------------------------------------------------
function [pot] = add_pot(x, y, th, row, col, pot)
[left right top bottom] = check_boundary(x, y, th, row, col);
[X Y] = gen_mesh(left, right, top, bottom, x, y);
r = sqrt(X.^2+Y.^2) + eps;
p = linear_possibility(r, th);
pot(top:bottom, left:right) = max(pot(top:bottom, left:right), p);
return;
%--------------------------------------------------------------------------
function [pot] = find_target_potential_field(row, col, target, th)
pot = zeros(row, col);
[left right top bottom] = check_boundary(target(1), target(2), th, row, ...
    col);
[X Y] = gen_mesh(left, right, top, bottom, target(1), target(2));
r = sqrt(X.^2+Y.^2) + eps;
p = linear_possibility(r, th);
pot(top:bottom, left:right) = max(pot(top:bottom, left:right), p);
return;
%--------------------------------------------------------------------------
function [y] = check_angle(x)
y = x;
if x>pi
    y = x - 2*pi;
elseif x<-pi
    y = x + 2*pi;
end
return;
%--------------------------------------------------------------------------
function [id] = find_obs(robot, target, color, map)
[X Y mapp] = draw_line(robot, target, color, map);
[row col depth] = size(map); 
id = (X-1)*col + Y;
tmp = map(:, :, 1);
id = intersect(id, find(tmp < 1));
return;
%--------------------------------------------------------------------------
function [Wx, Wy] = optimize_path(robot, X, Y, map)
n = length(X);
tmp_robot = robot;
Wx = [];
Wy = [];
i = 1;
while i<=n-1
    id = find_obs(tmp_robot, [X(i+1) Y(i+1)], [-1 0 0], map);
    if isempty(id)
        Wx = [Wx;X(i+1)];
        Wy = [Wy;Y(i+1)];
        tmp_robot = [X(i+1) Y(i+1)];
        i = i + 2;
    else
        Wx = [Wx;X(i)];
        Wy = [Wy;Y(i)];
        tmp_robot = [X(i) Y(i)];
        i = i + 1;
    end
end
Wx = [Wx; X(n)];
Wy = [Wy; Y(n)];
return;
%--------------------------------------------------------------------------
