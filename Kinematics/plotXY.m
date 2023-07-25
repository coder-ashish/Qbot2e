t=X.time;

x_ideal=X.signals.values(:,1);
y_ideal=Y.signals.values(:,1);


x_noGyro=X.signals.values(:,2);
y_noGyro=Y.signals.values(:,2);

plot(x_noGyro,y_noGyro);
hold on;

hold on;
plot(x_ideal,y_ideal,'k--');

grid on;
axis('square')
ylim([-0.5 1.5]);
xlim([-1 1]); 
