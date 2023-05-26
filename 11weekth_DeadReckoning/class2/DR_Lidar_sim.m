clear; clc; close all;
%% dead reckoning data initialize
load('CANLidar.mat');
RR = 0; RL = 0; Yaw_rate = 0;
x = 0; y = 0; th = deg2rad(180); vk = 0;
log_x(1) = 0;
log_y(1) = 0;
dt =0.1;
% calculation yaw mean during 5 sec
yaw_total = 0;
yaw_cnt = 50;
for i = 1:yaw_cnt
    yaw_total = yaw_total + log.yaw_rate(i);
end
yaw_mean = yaw_total/yaw_cnt;


figure();
for i = 1 : length(log.RR)
    % dead reckoning
    RL = log.RL(i); RR = log.RR(i); % vel of RL & RR (km/h) 
    Yaw_rate = log.yaw_rate(i) - yaw_mean; % remove drift
    vk = (RL+RR)/2/3.6; % x (km/h) = x*1000/3600(m/s) 
    [x, y, th] = dead_reckoning(x,y,th,vk,deg2rad(Yaw_rate),dt); 
    log_x = [log_x ; x];
    log_y = [log_y ; y];
    tf_Lidar = transform_Lidar(log.PointCloud(i).Location, x,y,th); 
    tf_car = transform_car(x,y,th);
    clf; hold on; grid on; axis equal; 
    plot3(tf_car(1,:),tf_car(2,:),tf_car(3,:),'b.','MarkerSize',0.1 ) 
    plot3(tf_Lidar(:,1),tf_Lidar(:,2),tf_Lidar(:,3),'.r','MarkerSize',0.4); 
    view([-60,30,55]);
    xlim([x-50 x+50]); ylim([y-50 y+50]); zlim([-5 35]);
    xlabel("x(m)"); ylabel('y(m)'); zlabel('z(m)');
    pause(0.001);
end
figure(); 
plot3(log_x,log_y,'bo','MarkerSize',5); hold on; axis equal;
xlim([-70 450]); ylim([-100 400]); zlim([-5 30]); xlabel("x(m)"); ylabel('y(m)'); zlabel('z(m)');