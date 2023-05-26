%% load Data
clear; clc;
load('DeadReckoning_data.mat');
RL = exp1_014.Y(4).Data; % rear left wheel speed
RR = exp1_014.Y(5).Data; % rear right wheel speed
gpsX = exp1_014.Y(11).Data-exp1_014.Y(11).Data(1);
gpsY = exp1_014.Y(12).Data-exp1_014.Y(12).Data(1);
Yaw_rate = exp1_014.Y(9).Data; % 각속도 : deg / sec
Time = exp1_014.X.Data; % sec

%% Display GPS
figure(1);
plot(gpsX, gpsY);

%% calc yaw_drift : 초기 상태의 mean을 구해서 전체 구간에 대해 drift를 보정해준다.
yaw_mean = mean(Yaw_rate(1:5944)); % estimation of yaw rate drift

%% initial Variable
x(1) = 0;
y(1) = 0;
th(1) = deg2rad(230);               % initial theta = 230 deg. 알고있는 차량의 각도
vk = (RL+RR)/2/3.6;                 % transform from kph to mps
wk = deg2rad(Yaw_rate-yaw_mean);    % remove drift

Ts(1) = 0.001;                      % data 받는 주기
for i = 2:length(Time)
    Ts(i) = Time(i) - Time(i-1);
end

%% (1) Dead Reckoning Using Euler
% initial variable
x_euler(1) = 0;
y_euler(1) = 0;
th_euler(1) = deg2rad(230);                 % initial theta = 230 deg. 알고있는 차량의 각도
vk_euler = (RL+RR)/2/3.6;                   % 속도 : Km/Hour
wk_euler = deg2rad(Yaw_rate - yaw_mean);    % 각속도, remove drift 

% Do Euler Integration 
for k =1:length(Time)
    x_euler(k+1) = x_euler(k) + vk_euler(k)*Ts(k)*cos(th_euler(k));     
    y_euler(k+1) = y_euler(k) + vk_euler(k)*Ts(k)*sin(th_euler(k));    
    th_euler(k+1) = th_euler(k) + ( wk_euler(k)*Ts(k) );
end
figure(2); plot(x_euler,y_euler);
title('Eular Dead Reckoning');

%% (2) Dead Reckoning Using 2nd Order Runge-Kutta
% initial variable
x_Runge(1) = 0;
y_Runge(1) = 0;
th_Runge(1) = deg2rad(230);                 % initial theta = 230 deg. 알고있는 차량의 각도
vk_Runge = (RL+RR)/2/3.6;                   % 속도 : Km/Hour
wk_Runge = deg2rad(Yaw_rate - yaw_mean);    % 각속도, remove drift

% Do Runge-Kutta Integration
for k = 1:length(Time)
    x_Runge(k+1) = x_Runge(k) + vk_Runge(k)*Ts(k)*cos(th_Runge(k) + (wk_Runge(k)*Ts(k)/2)); 
    y_Runge(k+1) = y_Runge(k) + vk_Runge(k)*Ts(k)*sin(th_Runge(k) + (wk_Runge(k)*Ts(k)/2)); 
    th_Runge(k+1) = th_Runge(k) + (wk_Runge(k)*Ts(k));
end
figure(3); plot(x_Runge,y_Runge);
title('Runge-kutta Dead Reckoning');

%% (3) Dead Reckoning Using Exact
% initial variable
x_Exact(1) = 0;
y_Exact(1) = 0;
th_Exact(1) = deg2rad(230);                   % initial theta = 230 deg. 알고있는 차량의 각도
vk_Exact = (RL+RR)/2/3.6;                     % 속도 : Km/Hour
wk_Exact = deg2rad(Yaw_rate - yaw_mean);      % 각속도, remove drift

% Do Exact Integration
for k = 1:length(Time)
    th_Exact(k+1) = th_Exact(k) + (wk_Exact(k)*Ts(k));
    x_Exact(k+1) = x_Exact(k) + vk_Exact(k)/wk(k)*(sin(th_Exact(k+1)) - sin(th_Exact(k))); 
    y_Exact(k+1) = y_Exact(k) - vk_Exact(k)/wk(k)*(cos(th_Exact(k+1)) - cos(th_Exact(k)));
end
figure(4); plot(x_Exact,y_Exact);
title('Exact Dead Reckoning');

%% Dead Reckoning Comparision
figure(5); hold on; 
plot(x_euler,y_euler,'b','LineWidth',2);            % Euler 
plot(x_Runge,y_Runge,'r','LineWidth',2);            % Runge-Kutta
plot(x_Exact,y_Exact,'k','LineWidth',2); grid on;   % Exact
legend('Euler','Runge-Kutta','Exact');
xlabel('Position x (m)'); 
ylabel ('Position y (m)');
title('DR comparison: Euler, Runge-Kutta & Exact');

%% Show Error of Velocity
err_exact_theta = x_Exact - x_euler; err_exact_y = y_Exact - y_euler;
err_runge_x = x_Exact - x_Runge; err_runge_y = y_Exact - y_Runge;

figure(6); clf;

subplot(211); hold on; 
yyaxis left;
plot([1:length(err_exact_theta)], err_exact_theta*1000,'b','LineWidth',2); 
plot([1:length(err_runge_x)], err_runge_x*1000,'k-','LineWidth',2);
yyaxis right;
plot([1:length(vk)], vk,'r-','LineWidth',0.5);                      % 실제 Car Velocity
xlabel('Sample Number');
yyaxis left; ylabel('x Difference (mm)');
yyaxis right; ylabel('Velocity (m/s)');
legend('Exact x - Euler x','Exact x - RungeKutta x','Car velocity'); 
title('DR differnces between Exact-Euler & Exact-RungeKutta'); 

subplot(212); hold on; 
yyaxis left
plot([1:length(err_exact_y)], err_exact_y*1000,'b','LineWidth',2); 
plot([1:length(err_runge_y)], err_runge_y*1000,'k-','LineWidth',2);
yyaxis right
plot([1:length(vk)], vk,'r-','LineWidth',0.5);
xlabel('Sample Number');
yyaxis left; ylabel('y Difference (mm)');
yyaxis right; ylabel('Velocity (m/s)');
legend('Exact x - Euler x','Exact x - RungeKutta x','Car velocity'); 

%% Show Error of Theta (\theta_{k+1} = \theta_k + w_k*T_s) 이므로 모두 똑같음 == 오차 0

err_euler_theta = th_Exact - th_euler;
err_runge_theta = th_Exact - th_Runge;

figure(6); clf;

hold on; 
yyaxis left;
plot([1:length(err_euler_theta)], err_euler_theta,'b-','LineWidth',5); 
plot([1:length(err_runge_theta)], err_runge_theta,'r-','LineWidth',2);
% yyaxis right;
% plot([1:length(th)], th,'r-','LineWidth',0.5);                      % 실제 Car Velocity
% xlabel('Sample Number');
% yyaxis left; ylabel('x Difference (mm)');
% yyaxis right; ylabel('Theta (degree)');
legend('Exact theta - Euler theta','Exact theta - RungeKutta theta','Car theta'); 
title('DR differnces between Exact-Euler & Exact-RungeKutta'); 

%% Show Error of Velocity 

err_euler_theta = th_Exact - th_euler;
err_runge_theta = th_Exact - th_Runge;

figure(6); clf;

hold on; 
yyaxis left;
plot([1:length(err_euler_theta)], err_euler_theta,'b-','LineWidth',5); 
plot([1:length(err_runge_theta)], err_runge_theta,'r-','LineWidth',2);
% yyaxis right;
% plot([1:length(th)], th,'r-','LineWidth',0.5);                      % 실제 Car Velocity
% xlabel('Sample Number');
% yyaxis left; ylabel('x Difference (mm)');
% yyaxis right; ylabel('Theta (degree)');
legend('Exact theta - Euler theta','Exact theta - RungeKutta theta','Car theta'); 
title('DR differnces between Exact-Euler & Exact-RungeKutta'); 

