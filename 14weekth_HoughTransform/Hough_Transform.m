%% Demonstration of Hough Transform

clear all; close all; figure(1); clf; grid on;
X = []; Y = [];

while (length(X) < 11)
    % figure(1) setting
    figure(1);
    set(gca,'Ydir','reverse'); set(gca,'XAxisLocation', 'top'); 
    xlim([0 100]); ylim([0 100]);
    %% x-y domain
    % x-y domain (x, y) input
    [x, y] = getpts;
    x = ceil(x); y = ceil(y);
    % x-y domain (x, y)를 list에 append
    X = [X; x]; 
    Y = [Y; y];
    % x-y domain plot
    clf;
    for i = 1:length(X)
        plot(X(i),Y(i),'.', 'MarkerSize', 20); hold on; grid on;
    end

    %% Hough domain
    % Hough domain으로 변환 (\theta, r)
    % x-y domain -> Hough domain 변환 공식 : r = x*cos(theta) + y*sin(theta)
    theta = -90:0.1:90;
    Rho = X*cos(theta*pi/180) + Y*sin(theta*pi/180);

     % Hough domain plot
    figure(2); clf; plot(theta,Rho); hold on; grid on;
    xlim([-91 91]); ylim([-sqrt(2)*100 sqrt(2)*100]);
end