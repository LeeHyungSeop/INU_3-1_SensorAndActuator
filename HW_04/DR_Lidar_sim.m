clear; clc; close all;

%% dead reckoning data initialize
load('CANLidar.mat');
RR = 0;                 % rear left wheel speed
RL = 0;                 % rear right wheel speed
Yaw_rate = 0;           % 각속도 : deg / sec
x = 0; y = 0;           % vehicle 초기 (x, y)
th = deg2rad(180);      % 각속도 : deg / sec
vk = 0;                 % 차량 초기 속도
log_x(1) = 0;           % DR 결과 x를 저장할 vector
log_y(1) = 0;           % DR 결과 y를 저장할 vector
dt = 0.1;               % 10Hz CAN 통신

%% Video

% calculation yaw mean during 5 sec  
sec = 5;
yaw_total = 0;
yaw_cnt = sec * dt;     % 50개 data (dt=0.1, sec=5)

% 전체 yaw data SUM
for i = 1:yaw_cnt
    yaw_total = yaw_total + log.yaw_rate(i);
end

% (전체 yaw data SUM / 개수) = (yaw data)의 mean 구함
yaw_mean = yaw_total/yaw_cnt;

figure()
for i = 1 : length(log.RR)  % 수신받은 data 개수만큼 반복
    % dead reckoning
    RL = log.RL(i); RR = log.RR(i);                         % vel of RL & RR (km/h) 
    Yaw_rate = log.yaw_rate(i) - yaw_mean;                  % remove drift
    vk = (RL+RR)/2/3.6;                                     % x (km/h) = x*1000/3600(m/s) 
    [x, y, th] = dead_reckoning(x,y,th,vk,deg2rad(Yaw_rate),dt);    % DR using Runge-Kutta Method
    log_x = [log_x ; x];                                    % DR을 통해 얻은 x값 저장
    log_y = [log_y ; y];                                    % DR을 통해 얻은 y값 저장
    tf_Lidar = transform_Lidar(log.PointCloud(i).Location, x,y,th); % Lidar data Parsing
    tf_car = transform_car(x,y,th);                         % Car State Transform

    % Car State, Lidar Data 그리기 (Video 한 Frame 출력)
    clf; hold on; grid on; axis equal; 
    plot3(tf_car(1,:),tf_car(2,:),tf_car(3,:),'b.','MarkerSize',0.1 ) 
    plot3(tf_Lidar(:,1),tf_Lidar(:,2),tf_Lidar(:,3),'.r','MarkerSize',0.4); 
    view([-60,30,55]);
    xlim([x-50 x+50]); ylim([y-50 y+50]); zlim([-5 35]);
    xlabel("x(m)"); ylabel('y(m)'); zlabel('z(m)');
    pause(0.001);
end

% Runge-Kutta DR을 사용한 전체 경로 그리기
figure(); 
plot(log_x,log_y,'bo','MarkerSize',5); hold on; axis equal;
xlim([-70 450]); ylim([-100 400]); zlim([-5 30]); 
xlabel("x(m)"); ylabel('y(m)'); zlabel('z(m)');