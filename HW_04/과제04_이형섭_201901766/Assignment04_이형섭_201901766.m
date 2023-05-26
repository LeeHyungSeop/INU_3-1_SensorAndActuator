%% load Data
clear; clc;
load('CANLidar.mat');
RL = log.RL;                % rear left wheel speed
RR = log.RR;                % rear right wheel speed
Yaw_rate = log.yaw_rate;    % 각속도 : deg / sec
Yaw_rate_len = length(Yaw_rate); 

dt = 0.1;       % CAN 통신 10Hz
Time = linspace(0, (Yaw_rate_len-1)/10, Yaw_rate_len); % sec


%% calc yaw_drift : 초기 상태의 mean을 구해서 전체 구간에 대해 drift를 보정해준다.

% 실습 1 : "DR_Lab_1a.m"에서 전체 178200개의 Yaw_rate data중에서 처음 5944개의 평균으로 보정.
yaw_mean_using_len = Yaw_rate_len * 0.033; % 5944 / 178200 = 약 0.033%
yaw_mean = mean(Yaw_rate(1 : yaw_mean_using_len)); % estimation of yaw rate drift

%% initial Variable
x(1) = 0;
y(1) = 0;
th(1) = deg2rad(180);               % initial theta = 180 deg. 알고있는 차량의 각도
vk = (RL+RR)/2/3.6;                 % transform from kph to mps
wk = deg2rad(Yaw_rate-yaw_mean);    % remove drift

Ts(1) = 0.1;                      % data 받는 주기
for i = 2:length(Time)
    Ts(i) = Time(i) - Time(i-1);
end

%% (1) Dead Reckoning Using Euler
% initial variable
x_euler(1) = 0;
y_euler(1) = 0;
th_euler(1) = deg2rad(180);                 % initial theta = 180 deg. 알고있는 차량의 각도
vk_euler = (RL+RR)/2/3.6;                   % 속도 : k/h -> m/s
wk_euler = deg2rad(Yaw_rate - yaw_mean);    % 각속도, remove drift 

% Do Euler Integration 
for k =1:length(Time)
    x_euler(k+1) = x_euler(k) + vk_euler(k)*Ts(k)*cos(th_euler(k));     
    y_euler(k+1) = y_euler(k) + vk_euler(k)*Ts(k)*sin(th_euler(k));    
    th_euler(k+1) = th_euler(k) + ( wk_euler(k)*Ts(k) );
end
figure(1); plot(x_euler,y_euler);
title('Eular Dead Reckoning');

%% (2) Dead Reckoning Using 2nd Order Runge-Kutta
% initial variable
x_Runge(1) = 0;
y_Runge(1) = 0;
th_Runge(1) = deg2rad(180);                 % initial theta = 180 deg. 알고있는 차량의 각도
vk_Runge = (RL+RR)/2/3.6;                   % 속도 : k/h -> m/s
wk_Runge = deg2rad(Yaw_rate - yaw_mean);    % 각속도, remove drift

% Do Runge-Kutta Integration
for k = 1:length(Time)
    x_Runge(k+1) = x_Runge(k) + vk_Runge(k)*Ts(k)*cos(th_Runge(k) + (wk_Runge(k)*Ts(k)/2)); 
    y_Runge(k+1) = y_Runge(k) + vk_Runge(k)*Ts(k)*sin(th_Runge(k) + (wk_Runge(k)*Ts(k)/2)); 
    th_Runge(k+1) = th_Runge(k) + (wk_Runge(k)*Ts(k));
end
figure(2); plot(x_Runge,y_Runge);
title('Runge-kutta Dead Reckoning');

%% (3) Dead Reckoning Using Exact
% initial variable
x_Exact(1) = 0;
y_Exact(1) = 0;
th_Exact(1) = deg2rad(180);                   % initial theta = 180 deg. 알고있는 차량의 각도
vk_Exact = (RL+RR)/2/3.6;                     % 속도 : k/h -> m/s
wk_Exact = deg2rad(Yaw_rate - yaw_mean);      % 각속도, remove drift

% Do Exact Integration
for k = 1:length(Time)
    th_Exact(k+1) = th_Exact(k) + (wk_Exact(k)*Ts(k));
    x_Exact(k+1) = x_Exact(k) + vk_Exact(k)/wk(k)*(sin(th_Exact(k+1)) - sin(th_Exact(k))); 
    y_Exact(k+1) = y_Exact(k) - vk_Exact(k)/wk(k)*(cos(th_Exact(k+1)) - cos(th_Exact(k)));
end
figure(3); plot(x_Exact,y_Exact);
title('Exact Dead Reckoning');

%% Dead Reckoning Comparision
figure(4); hold on; 
plot(x_euler,y_euler,'b','LineWidth',2);            % Euler 
plot(x_Runge,y_Runge,'r','LineWidth',2);            % Runge-Kutta
plot(x_Exact,y_Exact,'k','LineWidth',2); grid on;   % Exact
legend('Euler','Runge-Kutta','Exact');
xlabel('Position x (m)'); 
ylabel ('Position y (m)');
title('DR comparison: Euler, Runge-Kutta & Exact');

%% Show Error of X and Y

figure(5); clf;

% 1. x
err_euler_theta = x_Exact - x_euler;    % exact - euler (x)
err_runge_theta = x_Exact - x_Runge;    % exact - runge-kutta (x)
subplot(211); hold on;  
title('DR differnces between Exact-Euler & Exact-RungeKutta'); 
plot([1:length(err_euler_theta)], err_euler_theta*1000,'b','LineWidth',2); 
plot([1:length(err_runge_theta)], err_runge_theta*1000,'r','LineWidth',2);
xlabel('Sample Number');
ylabel('x Difference (mm)');
legend('Exact x - Euler x','Exact x - RungeKutta x'); 

% 2. y
err_euler_y = y_Exact - y_euler;    % exact - euler (y)
err_runge_y = y_Exact - y_Runge;    % exact - runge-kutta (y)
subplot(212); hold on; 
plot([1:length(err_euler_y)], err_euler_y*1000,'b','LineWidth',2); 
plot([1:length(err_runge_y)], err_runge_y*1000,'r','LineWidth',2);
xlabel('Sample Number');
ylabel('y Difference (mm)');
legend('Exact x - Euler x','Exact x - RungeKutta x'); 

%% Show Error of Theta and Velocity 

figure(6); clf;

% 3. Theta : (\theta_{k+1} = \theta_k + w_k*T_s) 이므로 모두 똑같음 == 오차 0
err_euler_theta = th_Exact - th_euler;    % exact - euler (theta)
err_runge_theta = th_Exact - th_Runge;    % exact - runge-kutta (theta)
subplot(211); hold on;  
title('DR differnces between Exact-Euler & Exact-RungeKutta'); 
plot([1:length(err_euler_theta)], err_euler_theta*1000,'b','LineWidth',5); 
plot([1:length(err_runge_theta)], err_runge_theta*1000,'r','LineWidth',3);
xlabel('Sample Number');
ylabel('theta Difference (rad)');
legend('Exact x - Euler x','Exact x - RungeKutta x'); 

% 4. Velocity 
% velocity는 DR의 3가지 방법에서 종속변수가 아니라 독립변수이기 때문에 모두 동일한 값을 가짐.
subplot(212); hold on; 
plot([1:length(vk)], vk,'r-','LineWidth',0.5);
xlabel('Sample Number');
ylabel('Velocity (m/s)');
legend('Car velocity'); 
