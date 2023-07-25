%clear all
close all
clc

boxwidth=1;
load Map_obs
%map = Map_1(130:170,130:170); % zoom in the desired area

map= Map_obs(100:200,100:200);
figure; 
imshow(map); 

hFig = figure(1);
set(hFig, 'Position', [820 20 800 800])

S=size(map);
 mapwidth=S(1);mapheight=S(2); maporgx=fix(mapwidth/2);maporgy=fix(mapheight/2);Rx=maporgx; Ry=maporgy; Tx= 200; Ty=20;


mapdisp=1*ones(S(2),S(2),3);
% using MATLAB functions this time for blob analysis
BW = im2bw(map, graythresh(map));
[B,L] = bwboundaries(BW,'holes');

marg = [1 1];

Label = zeros(size(L,1),size(L,2));


% down to C and Label

k=0;
for i = 1:length(B)
    [m n]= size(B{i});
    if (m*n > 40 && m*n < 300) % some limirtations on sizes to avoid background
        k=k+1;
        C{k}=B{i};
        Label(L==i)=k;
    end
end

%imshow(label2rgb(Label, @jet, [.5 .5 .5]))

n = length(C);
[m1 m2] = size(Label);

x= 1:m1;
y=1:m2;

%M= meshgrid(x,y);
M=reshape(1:m1*m2,m2,m1);


%% Create Occupancy_Grid MAP with Obstacles 
% Plot occupancy map ------------------------------------------------------
figure; 

X= [-mapwidth/2+1:mapwidth/2];
Y=[-mapheight/2+1:mapheight/2];

plot(X, zeros(1, mapwidth)), hold on
plot(zeros(1, mapwidth), Y)
axis([min(X) max(X) min(Y) max(Y)])

grid minor
title('Occupancy Grid Map');
xlabel('X_{robot}'); ylabel('Y_{robot}');

hFig = figure(2);
set(hFig, 'Position', [20 20 800 800])

% Plot obstacles ----------------------------------------------------------

obs = cell(n, 1);

for i=1:n
    clear r c;
    obsid = ['Obstacle ' num2str(i) ': '];
   % title(name);
    
    %index_obs= M(labeled==i);
    %c= fix(index_obs/m2);
    %r= mod(index_obs,m1);

    index_obs=C{i};
    Ymap = index_obs(:,1);
    Xmap= index_obs(:,2); % X and Y coordinates of the obstacle boundaries (in robot) == Y and X in the map
    
    
    Ymap(Ymap<5)=5;
    Xmap(Xmap<5)=5;
    
    xr= Ymap;
    yr = Xmap;
    

         % obstacle position in the robot coordinate frame
         Left_corner = [min(xr) min(yr)] - marg    - [maporgx maporgy] ;
         width = (max(xr) - min(xr)) + 2*marg(1); 
         height = ( max(yr) - min(yr)) + 2*marg(2) ; 

        x = Left_corner(1);
        y = Left_corner(2);


        coords = [x y;x+width y;x+width y+height;x y+height];
       
        %{
        id = find(coords(:, 1)>max(X));
        if ~isempty(id)
            coords(id, 1) = max(X);
        end
        id = find(coords(:, 2)>max(Y));
        if ~isempty(id)
            coords(id, 2) = max(Y);
        end
        %}
        
        %colorval = (i)/(n+1);
        colorval = 0;

        % AH: put some last checkes
        if (maporgy-coords(3,2)>0) && (maporgy-coords(1,2)>0) && (maporgx+coords(1,1)>0) && (maporgx+coords(3,1)>0) 
            mapdisp(maporgy-coords(3,2):maporgy-coords(1,2), ...
                maporgx+coords(1,1):maporgx+coords(3,1), 1) = colorval;
            mapdisp(-coords(3,2)+maporgy:maporgy-coords(1,2), ...
                maporgx+coords(1,1):maporgx+coords(3,1), 2) = colorval;
            mapdisp(-coords(3,2)+maporgy:maporgy-coords(1,2), ...
                maporgx+coords(1,1):maporgx+coords(3,1), 3) = colorval;

            obs{i} = [...
                maporgx+(coords(1,1)-1), maporgy-(coords(1,2)-1), 0;...
                maporgx+(coords(2,1)+1), maporgy-(coords(2,2)-1), 0;...
                maporgx+(coords(3,1)+1), maporgy-(coords(3,2)+1), 0;...
                maporgx+(coords(4,1)-1), maporgy-(coords(3,2)+1), 0];
        end
        px = coords(:, 1);
        py = coords(:, 2);
        color = [colorval colorval colorval];
        fill(px, py, color);


end

map_occupancy = mapdisp;

% Plot robot's position ---------------------------------------------------
title('Enter the initial position of the robot');
flag = 1;
while flag
    [rx ry] = ginput(1);  % in pix
    rx = round(rx);
    ry = round(ry);
    Rx = maporgx+rx;
    Ry = maporgy-ry;
    if mapdisp(Ry-boxwidth:Ry+boxwidth, Rx-boxwidth:Rx+boxwidth, 1) < 1
        disp('Robot''s position must not coincide with obstacles.');
    else
        mapdisp(Ry-boxwidth:Ry+boxwidth, Rx-boxwidth:Rx+boxwidth, 2) = 0;
        mapdisp(Ry-boxwidth:Ry+boxwidth, Rx-boxwidth:Rx+boxwidth, 3) = 0;

        coords = [rx-boxwidth ry+boxwidth;...
            rx-boxwidth ry-boxwidth;...
            rx+boxwidth ry-boxwidth;...
            rx+boxwidth ry+boxwidth];
        px = coords(:, 1);
        py = coords(:, 2);
        robotcolor = [1 0 0];
        fill(px, py, robotcolor);
        
      %  title(maptitle);
        flag = 0;
    end
end

% Plot target's position --------------------------------------------------
title('Enter the target position');
flag = 1;
while flag
    [tx ty] = ginput(1);  % in pix
    tx = round(tx);
    ty = round(ty);
    Tx = maporgx+tx;
    Ty = maporgy-ty;
    if mapdisp(Ty-boxwidth:Ty+boxwidth, Tx-boxwidth:Tx+boxwidth, 1) < 1
        disp('target''s position must not coincide with obstacles.');
    else
        mapdisp(Ty-boxwidth:Ty+boxwidth, Tx-boxwidth:Tx+boxwidth, 1) = 0;
        mapdisp(Ty-boxwidth:Ty+boxwidth, Tx-boxwidth:Tx+boxwidth, 2) = 0;

        coords = [tx-boxwidth ty+boxwidth;...
            tx-boxwidth ty-boxwidth;...
            tx+boxwidth ty-boxwidth;...
            tx+boxwidth ty+boxwidth];
        px = coords(:, 1);
        py = coords(:, 2);
        targetcolor = [0 0 1];
        fill(px, py, targetcolor);
        
   %     title(maptitle);
        flag = 0;
    end
end
% hold off

%map_occupancy = mapdisp;

save map_occupancy


