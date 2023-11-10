clear all; close all;
L = 2.7; % L = wheelbase (m)

steering_angle = [2, 5, 10, 15, 20, 30]; % deg
steering_angle_rad = deg2rad(steering_angle); % degree -> radian

R = L ./ steering_angle_rad;
Delta = R.*(1 - cos(steering_angle_rad))

figure(1);clf;
plot(steering_angle, Delta,'-ro');
xlabel('steering angle (deg)');
ylabel('off-tracking distance \Delta (m)');
grid on; grid minor; box on;