%% load Data
clear; clc;
load('DeadReckoning_data.mat');
RL = exp1_014.Y(4).Data; % rear left wheel speed
RR = exp1_014.Y(5).Data; % rear right wheel speed
gpsX = exp1_014.Y(11).Data-exp1_014.Y(11).Data(1);
gpsY = exp1_014.Y(12).Data-exp1_014.Y(12).Data(1);
Yaw_rate = exp1_014.Y(9).Data; % deg / sec
Time = exp1_014.X.Data; % sec

%% calc yaw_drift
yaw_mean = mean(Yaw_rate(1:5944)); % estimation of yaw rate drift
% 초기 상태의 mean을 구해서 전체 구간에 대해 drift를 보정해준다.
%% initial Variable
x(1) = 0;
y(1) = 0;
th(1) = deg2rad(230); % initial theta = 230 deg. 알고있는 차량의 각도
vk = (RL+RR)/2/3.6; % transform from kph to mps
wk = deg2rad(Yaw_rate-yaw_mean); % remove drift
Ts(1) = 0.001;
for i = 2:length(Time)
    Ts(i) = Time(i) - Time(i-1);
end
%% Display GPS
figure();
plot(gpsX, gpsY);
%% Dead Reckoning Using Euler
x_euler(1) = 0;
y_euler(1) = 0;
th_euler(1) = deg2rad(230);
vk_euler = (RL+RR)/2/3.6;
wk_euler = deg2rad(Yaw_rate - yaw_mean);
for k =1:length(Time)
    x_euler(k+1) = x_euler(k) + vk_euler(k)*Ts(k)*cos(th_euler(k)); 
    y_euler(k+1) = y_euler(k) + vk_euler(k)*Ts(k)*sin(th_euler(k)); 
    th_euler(k+1) = th_euler(k) + ( wk_euler(k)*Ts(k) );
end
figure(); plot(x_euler,y_euler);
title('Eular Dead Reckoning');

%% Dead Reckoning Using 2nd Order Runge-Kutta
x_Runge(1) = 0;
y_Runge(1) = 0;
th_Runge(1) = deg2rad(230);
vk_Runge = (RL+RR)/2/3.6;
wk_Runge = deg2rad(Yaw_rate - yaw_mean);
% transform from kph to mps
% remove drift
for k = 1:length(Time)
    x_Runge(k+1) = x_Runge(k) + vk_Runge(k)*Ts(k)*cos(th_Runge(k) + (wk_Runge(k)*Ts(k)/2)); 
    y_Runge(k+1) = y_Runge(k) + vk_Runge(k)*Ts(k)*sin(th_Runge(k) + (wk_Runge(k)*Ts(k)/2)); 
    th_Runge(k+1) = th_Runge(k) + (wk_Runge(k)*Ts(k));
end
figure(); plot(x_Runge,y_Runge);
title('Runge-kutta Dead Reckoning');
%% Dead Reckoning Using Exact
x_Exa(1) = 0;
y_Exa(1) = 0;
th_Exa(1) = deg2rad(230);
vk_Exa = (RL+RR)/2/3.6;
wk_Exa = deg2rad(Yaw_rate - yaw_mean);
% transform from kph to mps
% remove drift
for k = 1:length(Time)
    th_Exa(k+1) = th_Exa(k) + (wk_Exa(k)*Ts(k));
    x_Exa(k+1) = x_Exa(k) + vk_Exa(k)/wk(k)*(sin(th_Exa(k+1)) - sin(th_Exa(k))); 
    y_Exa(k+1) = y_Exa(k) - vk_Exa(k)/wk(k)*(cos(th_Exa(k+1)) - cos(th_Exa(k)));
end
figure(); % DR comparision figure 
plot(x_euler,y_euler,'b','LineWidth',2); hold on; 
plot(x_Runge,y_Runge,'r','LineWidth',2); 
plot(x_Exa,y_Exa,'k','LineWidth',2); grid on; 
legend('Euler','Runge-Kutta','Exact');
xlabel('Position x (m)'); ylabel ('Position y (m)');
title('DR comparison: Euler, Runge-Kutta & Exact');

%%

Ee_x = x_Exa - x_euler; Ee_y = y_Exa - y_euler;
ER_x = x_Exa - x_Runge; ER_y = y_Exa - y_Runge;
figure(); clf;subplot(211);
yyaxis left
plot([1:length(Ee_x)], Ee_x*1000,'b','LineWidth',2); hold on; plot([1:length(ER_x)], ER_x*1000,'k-','LineWidth',2);
yyaxis right
plot([1:length(vk)], vk,'r-','LineWidth',0.5);
legend('Exact x - Euler x','Exact x - RK x','Car velocity'); title('DR differnces between Exact-Euler & Exact-RK'); yyaxis left
xlabel('Sample Number'), ylabel('x Difference (mm)');
yyaxis right; ylabel('Velocity (m/s)');
subplot(212);
yyaxis left
plot([1:length(Ee_y)], Ee_y*1000,'b','LineWidth',2); hold on; plot([1:length(ER_y)], ER_y*1000,'k-','LineWidth',2);
yyaxis right
plot([1:length(vk)], vk,'r-','LineWidth',0.5);
legend('Exact x - Euler x','Exact x - RK x','Car velocity'); yyaxis left
xlabel('Sample Number'), ylabel('y Difference (mm)');
yyaxis right; ylabel('Velocity (m/s)');