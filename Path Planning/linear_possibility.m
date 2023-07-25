function y = linear_possibility(x, max_x)
x = abs(x);
max_x = abs(max_x);
id = find(x>max_x);
x(id) = max_x;
y = 1 - (x/max_x);
return;
