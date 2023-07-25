function [X, Y, mapout] = draw_line(p1, p2, color, mapin)
% [X, Y, map] = draw_line(p1, p2, color, map)
% This function takes two input points and draw a line
% between the points. 
% Inputs:
%   p1 = [x1 y1], where (x1, y1) is the location of point 1. Note that x1 
%                 and y1 must be integer values.
%   p2 = [x2 y2], where (x2, y2) is the location of point 2. Note that x2 
%                 and y2 must be integer values.
%   color = [R G B], where color values must be 0 to 1 for red, green and,
%                 blue color planes.
%   mapin = A 3D image, where pixel values must be 0 to 1.
% Outputs:
%   X = A vector of x coordinates of the points lying on the line
%   Y = A vector of y coordinates of the points lying on the line
%   mapout = Updated map with the line drawn between point 1 and 2 using the 
%         specified color.
% Example:
%   p1 = [20 20]; 
%   p2 = [220 120];
%   color = [1 0 0];  
%   width = 255; height = 255;
%   buff = ones(height, width);
%   map(:, :, 1) = buff;
%   map(:, :, 2) = buff;
%   map(:, :, 3) = buff;
%   [map X Y] = draw_line(p1, p2, color, map);
%   image(map)

x1 = p1(1);
y1 = p1(2);
x2 = p2(1);
y2 = p2(2);
X = [];
Y = [];
mapout = mapin;

step_x = 1;
step_y = 1;

if x1>x2
    step_x = -1;
end
if y1>y2
    step_y = -1;
end
m = (y2-y1)/(x2-x1);
for x=x1:step_x:x2
    y = round(y1 + m*(x-x1));
    X = [X;x];
    Y = [Y;y];
end
for y=y1:step_y:y2
    x = round(x1 + (1/m)*(y-y1));
    X = [X;x];
    Y = [Y;y];
end

tmp1 = [X Y];
tmp2 = [X Y];
tmp = intersect(tmp1, tmp2, 'rows');
X = tmp(:, 1);
Y = tmp(:, 2);
if length(size(color))==2 && size(color,1)==1 && size(color, 2)==3
    maxval = max(color);
    minval = min(color);
    if maxval>1 || minval<0
%         disp('Invalid color values.');
        return;
    end
end

mapout = draw_point(X, Y, mapout, color);

return;

function [map] = draw_point(X, Y, map, color)


height = size(map, 1);
width = size(map, 2);
depth = size(map, 3);

for i=1:depth
    for j=1:length(X)
        if (X(j)>=1 && X(j)<=width) && (Y(j)>=1 && Y(j)<=height)
            map(Y(j), X(j), i) = color(i);
        end
    end
end
    
return;
