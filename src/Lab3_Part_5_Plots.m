data = csvread('3D_plot.txt');
%% plot 1

plot3(data(:,1),data(:,2),data(:,3));
title('Task Space Values');
grid on

%% plot 2
plot(data(:,4),data(:,1));
hold on
plot(data(:,4),data(:,2));
plot(data(:,4),data(:,3));
hold off 
title('Positon vs Time');
xlabel('time in seconds');
ylabel('distance in mm');
legend('x','y','z');
%% plot 3

dx = data(2:end,1)-data(1:end-1,1);
dy = data(2:end,2)-data(1:end-1,2);
dz = data(2:end,3)-data(1:end-1,3);
dt = data(2:end,4)-data(1:end-1,4);
vx = dx./dt;
vy = dy./dt;
vz = dz./dt;
plot(data(1:end-1,4),vx);
hold on
plot(data(1:end-1,4),vy);
plot(data(1:end-1,4),vz);
hold 
title('Velocity Vs Time');
xlabel('time in seconds');
ylabel('velocity mm/s');
legend('x','y','z');
%% plot 3 tentative
dtdx = gradient(data(:,4)) ./ gradient(data(:,1));
dtdy = gradient(data(:,4)) ./ gradient(data(:,2));
dtdz = gradient(data(:,4)) ./ gradient(data(:,3));
plot(data(:,4),dtdx);
hold on
plot(data(:,4),dtdy);
plot(data(:,4),dtdz);
hold off
legend('x','y','z');
%% plot 4
ax = dx(2:end)-dx(1:end-1);
ay = dy(2:end)-dy(1:end-1);
az = dz(2:end)-dz(1:end-1);
dt = data(2:end-1,4)-data(1:end-2,4);
plot(data(1:end-2,4),ax);
hold on
plot(data(1:end-2,4),ay);
plot(data(1:end-2,4),az);
hold off
title('Acceleration Vs Time');
xlabel('time in seconds');
ylabel('acceleration mm/s');
legend('x','y','z');

%% 
data = csvread('3D_plot_54.txt');
%% plot 1

plot3(data(:,1),data(:,2),data(:,3));
title('Task Space Values');
grid on

%% plot 2
plot(data(:,4),data(:,1));
hold on
plot(data(:,4),data(:,2));
plot(data(:,4),data(:,3));
hold off 
title('Positon vs Time');
xlabel('time in seconds');
ylabel('distance in mm');
legend('x','y','z');
%% plot 3

dx = data(2:end,1)-data(1:end-1,1);
dy = data(2:end,2)-data(1:end-1,2);
dz = data(2:end,3)-data(1:end-1,3);
dt = data(2:end,4)-data(1:end-1,4);
vx = dx./dt;
vy = dy./dt;
vz = dz./dt;
plot(data(1:end-1,4),vx);
hold on
plot(data(1:end-1,4),vy);
plot(data(1:end-1,4),vz);
hold off
title('Velocity Vs Time');
xlabel('time in seconds');
ylabel('velocity mm/s');
legend('x','y','z');
%% plot 3 tentative
dtdx = gradient(data(:,4)) ./ gradient(data(:,1));
dtdy = gradient(data(:,4)) ./ gradient(data(:,2));
dtdz = gradient(data(:,4)) ./ gradient(data(:,3));
plot(data(:,4),dtdx);
hold on
plot(data(:,4),dtdy);
plot(data(:,4),dtdz);
hold off
legend('x','y','z');
%% plot 4
ax = dx(2:end)-dx(1:end-1);
ay = dy(2:end)-dy(1:end-1);
az = dz(2:end)-dz(1:end-1);
dt = data(2:end-1,4)-data(1:end-2,4);
plot(data(1:end-2,4),ax);
hold on
plot(data(1:end-2,4),ay);
plot(data(1:end-2,4),az);
hold off
title('Acceleration Vs Time');
xlabel('time in seconds');
ylabel('Acceleration mm/s^2');
legend('x','y','z');
