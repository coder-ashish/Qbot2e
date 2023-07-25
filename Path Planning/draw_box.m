function [map] = draw_box(x, y, width, height, color, map)
map(y-height:y+height, x-width:x+width, 1) = color(1);
map(y-height:y+height, x-width:x+width, 2) = color(2);
map(y-height:y+height, x-width:x+width, 3) = color(3);
return;
