%% load Data
clear; clc;
load('DeadReckoning_data.mat');
                                % 앞 바퀴의 속도만 고려한다. 왜? ...
RL = exp1_014.Y(4).Data;        % 앞 왼쪽 바퀴 속도
RR = exp1_014.Y(5).Data;        % 앞 오른쪽 바퀴 속도

gpsX = exp1_014.Y(11).Data-exp1_014.Y(11).Data(1);
gpsY = exp1_014.Y(12).Data-exp1_014.Y(12).Data(1);
Yaw_rate = exp1_014.Y(9).Data;  % sensor는 bias가 쉽게 발생한다. 또는 drift(sensor에 의해 발생하는 값이 뜨는 커지는 현상)
Time = exp1_014.X.Data;

%% calc yaw_drift
yaw_mean = mean(Yaw_rate(1:5944));   % sensor data의 bias, dift 현상으로 인해, 초반 5944개의 sample의 평균을 구한다.
                                     % 초반 5944개의 평균만 구한 이유? 
                                     % sensor가 갖고 있는 dift가 얼만지를 계산하기 위해
                                     
%% initial Variable
x(1) = 0;                 % x : unknown
y(1) = 0;                 % y : unknown
th(1) = deg2rad(230);     % theta = 230 (deg) : known
vk = (RL+RR)/2/3.6;         % m/s 단위로 바꿔주기 위해, 뒷바퀴 두개의 속도를 평균 -> 차량의 wheel speed라고 판단
wk = deg2rad(Yaw_rate-yaw_mean); % 휠 속도는 

Ts(1) = 0.001;
for i = 2:length(Time)
    Ts(i) = Time(i) - Time(i-1);
end

%% Display GPS
figure();
plot(gpsX, gpsY);

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
