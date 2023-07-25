function d_est = particle_sense(x,y,psi,k_angles,M)
%#codegen
S=300;
scale=10;

steps=0:1:100; % Kinect cannot give us higher than 10 meter
%r_s=20:10:620;
%k_angles_resampled=k_angles(r_s);

k_angles_resampled=resample_data(k_angles);
sweep_psi=psi-k_angles_resampled;

d=100*ones(1,length(k_angles_resampled));

x_c=round(scale*x);
y_c=round(scale*y);

for k=1:length(sweep_psi)
    L_x=x_c+S/2+1+round (steps*cos(sweep_psi(k)));
    L_y=y_c+S/2+1+round (steps*sin(sweep_psi(k)));

    L_x(L_x>length(M)-1)=length(M)-1;
    L_y(L_y>length(M)-1)=length(M)-1;

    L_x(L_x<1)=1;
    L_y(L_y<1)=1;
    
    for i=length(L_x):-1:2
        if (M(L_x(i),L_y(i))<10) % this is an obstacle
            d(k)=sqrt((L_x(i)-(x_c+S/2+1))^2+(L_y(i)-(y_c+S/2+1))^2);
        end
    end
end

d_est=d/scale;
%dist_resampled=dist(r_s);