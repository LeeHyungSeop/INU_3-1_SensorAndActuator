clear; clc; close all;

%% Dead reckoning data initialize
load('CANLidar.mat');
RR = 0; RL = 0; Yaw_rate = 0;
x = 0; y = 0; th = deg2rad(180); vk = 0;
log_x(1) = 0;
log_y(1) = 0;
dt = 0.1;

% Calculate mean yaw during 5 sec
yaw_mean = mean(log.yaw_rate(1:50));

% Arrays to store errors
error_x_euler = [];
error_y_euler = [];
error_velocity_euler = [];
error_theta_euler = [];

error_x_rk = [];
error_y_rk = [];
error_velocity_rk = [];
error_theta_rk = [];

figure();
for i = 1:length(log.RR)
    % Dead reckoning
    RL = log.RL(i);
    RR = log.RR(i);
    Yaw_rate = log.yaw_rate(i) - yaw_mean;
    vk = (RL + RR) / 2 / 3.6;

    % Exact method
    [x_exact, y_exact, th_exact] = dead_reckoning(x, y, th, vk, deg2rad(Yaw_rate), dt);

    % Euler method
    [x_euler, y_euler, th_euler] = dead_reckoning(x, y, th, vk, deg2rad(Yaw_rate), dt);

    % Runge-Kutta method
    [x_rk, y_rk, th_rk] = dead_reckoning2(x, y, th, vk, deg2rad(Yaw_rate), dt);

    % Update previous values
    x = x_exact;
    y = y_exact;
    th = th_exact;

    % Store positions
    log_x = [log_x, x_exact];
    log_y = [log_y, y_exact];

    % Store errors
    error_x_euler = [error_x_euler; x_exact - x_euler];
    error_y_euler = [error_y_euler; y_exact - y_euler];
    error_velocity_euler = [error_velocity_euler; vk - sqrt(x_euler^2 + y_euler^2)];
    error_theta_euler = [error_theta_euler; th_exact - th_euler];

    error_x_rk = [error_x_rk; x_exact - x_rk];
    error_y_rk = [error_y_rk; y_exact - y_rk];
    error_velocity_rk = [error_velocity_rk; vk - sqrt(x_rk^2 + y_rk^2)];
    error_theta_rk = [error_theta_rk; th_exact - th_rk];

    tf_Lidar = transform_Lidar(log.PointCloud(i).Location, x_exact, y_exact, th_exact);
    tf_car = transform_car(x_exact, y_exact, th_exact);

%     clf; hold on; grid on; axis equal;
%     plot3(tf_car(1, :), tf_car(2, :), tf_car(3, :), 'b.', 'MarkerSize', 0.1);
%     plot3(tf_Lidar(:, 1), tf_Lidar(:, 2), tf_Lidar(:, 3), '.r', 'MarkerSize', 0.4);
%     view([-60, 30, 55]);
%     xlim([x_exact - 50, x_exact + 50]);
%     ylim([y_exact - 50, y_exact + 50]);
%     zlim([-5, 35]);
%     xlabel("x(m)");
%     ylabel('y(m)');
%     zlabel('z(m)');
%     pause(0.001);
end

figure();
plot(log_x, log_y, 'bo', 'MarkerSize', 5);
hold on;
axis equal;
xlim([-70 450]);
ylim([-100 400]);
zlim([-5 30]);
xlabel("x(m)");
ylabel('y(m)');
zlabel('z(m)');
title('Exact Dead Reckoning');

% Plotting errors for Euler method
figure();
subplot(4, 1, 1);
plot(1:length(error_x_euler), error_x_euler*1000, 'b-', 'LineWidth', 2); hold on;
plot(1:length(error_x_rk), error_x_rk*1000, 'k-', 'LineWidth', 2);
legend('Exact x - Euler x', 'Exact x - RK x');
title('Dead Reckoning Differences between Exact-Euler & Exact-RK');
ylabel('x Difference (m)');

subplot(4, 1, 2);
plot(1:length(error_y_euler), error_y_euler*1000, 'b-', 'LineWidth', 2); hold on;
plot(1:length(error_y_rk), error_y_rk*1000, 'k-', 'LineWidth', 2);
legend('Exact y - Euler y', 'Exact y - RK y');
ylabel('y Difference (m)');

subplot(4, 1, 3);
plot(1:length(error_velocity_euler), error_velocity_euler, 'b-', 'LineWidth', 2); hold on;
plot(1:length(error_velocity_rk), error_velocity_rk, 'k-', 'LineWidth', 2);
legend('Exact velocity - Euler velocity', 'Exact velocity - RK velocity');
ylabel('Velocity Difference (m/s)');

subplot(4, 1, 4);
plot(1:length(error_theta_euler), error_theta_euler, 'b-', 'LineWidth', 2); hold on;
plot(1:length(error_theta_rk), error_theta_rk, 'k-', 'LineWidth', 2);
legend('Exact theta - Euler theta', 'Exact theta - RK theta');
xlabel('Time step');
ylabel('Theta Difference (rad)');