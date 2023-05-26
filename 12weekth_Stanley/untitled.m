clc; clear all; close all;

%% Vehicle information
x_v = 0;                % initial vehicle x position (m)
y_v = 0;                % initial vehicle y position (m)
theta_vehicle = deg2rad(0);     % initial vehicle theta (rad)
L = 2.7;                % wheelbase (m)
v = 3;                  % vehicle speed (m/s, constant)

%% Steering constraints
steer_limit = deg2rad(40.95); % 조향각도limit :-40.95<x<40.95 deg
steer_rate_limit = deg2rad(24); % 조향각속도 limit : 24 deg/s
cmd_sta = deg2rad(0);     % initial vehicle steering angle(deg)

%% target path : waypoint setting
Amplitude = 3;          % waypoint amplitude (m)
Wavelength = 25;        % wave_length (m)
offset = -5;            % offset of path (m);
N_wave = 1.75;	% wave cycle number = 1.75 cycle

path_x = 0:0.1:40;
path_y = Amplitude*sin(2*pi/Wavelength * path_x) + offset;
%% 밑그림 그리기
shape_vx = [-3.63, 0.93, 0.93, -3.63]; % 직4각형 차량 모서리 x좌표 (m)
shape_vy = [0.9 0.9 -0.9 -0.9];	   % 직4각형 차량 모서리 y좌표 (m)
shape_fx = [-0.32 0.32 0.32 -0.32];    % 직4각형 앞바퀴 x좌표 (m)
shape_fy = [0.20 0.20 -0.20 -0.20];    % 직4각형 앞바퀴 y좌표 (m)
shape_rx = [-0.32 0.32 0.32 -0.32];    % 직4각형 뒷바퀴 x좌표 (m)
shape_ry = [0.20 0.20 -0.20 -0.20];    % 직4각형 뒷바퀴 y좌표 (m)

fd = [0, 0];    % 차 앞바퀴 중심점 (원점)
dfx = cos(theta_vehicle)*fd(1) - sin(theta_vehicle)*fd(2);
dfy = sin(theta_vehicle)*fd(1) + cos(theta_vehicle)*fd(2);
rd = [-2.7, 0];   % 차 뒷바퀴 중심점
drx = cos(theta_vehicle)*rd(1) - sin(theta_vehicle)*rd(2);
dry = sin(theta_vehicle)*rd(1) + cos(theta_vehicle)*rd(2);

dd = [fd;rd];
log_fx = zeros(1,2);                    
log_rx = zeros(1,2);
cx1 = path_x(1); cy1 = path_y(1);

figure(); hold on;
vehicle = fill(shape_vx,shape_vy,'w','EraseMode','normal');
fw = fill(shape_fx,shape_fy,'r','EraseMode','normal');
rw = fill(shape_rx,shape_ry,'b','EraseMode','normal');
pd = plot(dd(:,1),dd(:,2),'k','LineWidth',2);
pfw = plot(log_fx(:,1),log_fx(:,2),'r.');
prw = plot(log_rx(:,1),log_rx(:,2),'b.');
pl3 = plot(path_x,path_y,'k.');
pl4 = fplot(@(x1) x1,'r');
pl5 = fplot(@(x1) x1,'b');
pl6 = plot(cx1,cy1,'go'); 



j = 0; prev_sta_rad = 0;
k = 0.5;
sim_time = ceil(N_wave*4/v * sqrt(Amplitude^2+(Wavelength/4)^2));
dt = 0.01;

for i = 0:dt:sim_time
    idx = find_closest_point(x_v,y_v,path_x,path_y);
    cx1 = path_x(idx); % closest waypoint x position 
    cy1 = path_y(idx); % closest waypoint y position 
    if (idx == length(path_x))
        break; % check end condition 
    else
        cx2 = path_x(idx+1);   % next waypoint x position
        cy2 = path_y(idx+1);   % next waypoint y position
    end

%% 
% p_c을 지나는 방정식 fn_path = slope_path(x-cx1)+cy1
slope_path = (cy2-cy1)/(cx2-cx1); % 경로의 기울기 
fn_path = @(x1) slope_path * (x1 - cx1) + cy1;
% X, 이탈거리
% 직선방정식과 차량의 앞바퀴 점 사이의 거리
X = abs(slope_path * x_v - y_v +(-slope_path*cx1 + cy1)) / sqrt(slope_path^2 + 1);
%% 
theta_path   = atan2(cy2- cy1, cx2- cx1);
leteral_flag = (x_v-cx1)*sin(-theta_path) + (y_v-cy1)*cos(-theta_path);
if leteral_flag >= 0
    X = -X;
end
%%
% Calculation alpha
   alpha = atan2(k*X,v);
%% Calculation theta error
A = (dfy-dry)/(dfx-drx);
% 차량의 진행방향(yaw) 직선방정식
Vehicle_heading = @(x2) A * (x2 - drx - x_v) + dry + y_v;
theta_path = atan2(cy2-cy1, cx2-cx1);
theta_e = theta_path - theta_vehicle;
sta_cmd_rad = theta_e + alpha;

%% 6
%% Check constraints
%------------steering 각속도 Limit------------%
if abs(sta_cmd_rad-prev_sta_rad)> steer_rate_limit*dt 
    if sta_cmd_rad > prev_sta_rad
       sta_cmd_rad = prev_sta_rad + steer_rate_limit*dt;
    else
       sta_cmd_rad = prev_sta_rad - steer_rate_limit*dt;
    end
end %---------------Steering_angle_Limit---------------% 
if sta_cmd_rad > steer_limit
   sta_cmd_rad = steer_limit;
elseif sta_cmd_rad < -steer_limit
   sta_cmd_rad = -steer_limit;
end
prev_sta_rad = sta_cmd_rad;
cmd_sta = sta_cmd_rad;

%% Update vehicle 
    x_v = x_v + v * cos( theta_vehicle + cmd_sta ) * dt;
    y_v = y_v + v * sin( theta_vehicle + cmd_sta ) * dt;
    theta_vehicle = theta_vehicle + v * tan(cmd_sta)/L*dt;
    
    updatedFx=[shape_fx(1)*cos(theta_vehicle+cmd_sta)-shape_fy(1)*sin(theta_vehicle+cmd_sta)+ dfx + x_v
        shape_fx(2) * cos(theta_vehicle+cmd_sta) - shape_fy(2) * sin(theta_vehicle+cmd_sta) + dfx + x_v
        shape_fx(3) * cos(theta_vehicle+cmd_sta) - shape_fy(3) * sin(theta_vehicle+cmd_sta) + dfx + x_v
        shape_fx(4) * cos(theta_vehicle+cmd_sta) - shape_fy(4) * sin(theta_vehicle+cmd_sta) + dfx + x_v];
    updatedFy =[shape_fx(1)*sin(theta_vehicle+cmd_sta)+shape_fy(1)*cos(theta_vehicle+cmd_sta)+dfy + y_v
        shape_fx(2) * sin(theta_vehicle+cmd_sta) + shape_fy(2) * cos(theta_vehicle+cmd_sta) + dfy + y_v
        shape_fx(3) * sin(theta_vehicle+cmd_sta) + shape_fy(3) * cos(theta_vehicle+cmd_sta) + dfy + y_v
        shape_fx(4) * sin(theta_vehicle+cmd_sta) + shape_fy(4) * cos(theta_vehicle+cmd_sta) + dfy + y_v];
    
    updatedRx = [shape_rx(1) * cos(theta_vehicle) - shape_ry(1) * sin(theta_vehicle) + drx + x_v
        shape_rx(2) * cos(theta_vehicle) - shape_ry(2) * sin(theta_vehicle) + drx + x_v
        shape_rx(3) * cos(theta_vehicle) - shape_ry(3) * sin(theta_vehicle) + drx + x_v
        shape_rx(4) * cos(theta_vehicle) - shape_ry(4) * sin(theta_vehicle) + drx + x_v];
    updatedRy = [shape_rx(1) * sin(theta_vehicle) + shape_ry(1) * cos(theta_vehicle) + dry + y_v
        shape_rx(2) * sin(theta_vehicle) + shape_ry(2) * cos(theta_vehicle) + dry + y_v
        shape_rx(3) * sin(theta_vehicle) + shape_ry(3) * cos(theta_vehicle) + dry + y_v
        shape_rx(4) * sin(theta_vehicle) + shape_ry(4) * cos(theta_vehicle) + dry + y_v];
    updatedVx = [shape_vx(1) * cos(theta_vehicle) - shape_vy(1) * sin(theta_vehicle) + x_v
        shape_vx(2) * cos(theta_vehicle) - shape_vy(2) * sin(theta_vehicle) + x_v
        shape_vx(3) * cos(theta_vehicle) - shape_vy(3) * sin(theta_vehicle) + x_v
        shape_vx(4) * cos(theta_vehicle) - shape_vy(4) * sin(theta_vehicle) + x_v];
    updatedVy = [shape_vx(1) * sin(theta_vehicle) + shape_vy(1) * cos(theta_vehicle) + y_v
        shape_vx(2) * sin(theta_vehicle) + shape_vy(2) * cos(theta_vehicle) + y_v
        shape_vx(3) * sin(theta_vehicle) + shape_vy(3) * cos(theta_vehicle) + y_v
        shape_vx(4) * sin(theta_vehicle) + shape_vy(4) * cos(theta_vehicle) + y_v];
    

        dfx = cos(theta_vehicle)*fd(1) - sin(theta_vehicle)*fd(2);
    dfy = sin(theta_vehicle)*fd(1) + cos(theta_vehicle)*fd(2);
    drx = cos(theta_vehicle)*rd(1) - sin(theta_vehicle)*rd(2);
    dry = sin(theta_vehicle)*rd(1) + cos(theta_vehicle)*rd(2);
    dd = [dfx+x_v, dfy+y_v ; drx+x_v , dry+y_v ];
    j = j+1;
    log_fw(j,:) = dd(1,:);
    log_rw(j,:) = dd(2,:);
    
    set(vehicle, 'Xdata', updatedVx,'Ydata', updatedVy); 
    set(fw, 'Xdata', updatedFx,'Ydata', updatedFy); 
    set(rw, 'Xdata', updatedRx,'Ydata', updatedRy);
    set(pd, 'Xdata', dd(:,1),'Ydata',dd(:,2)); 
    set(pfw, 'Xdata', log_fw(:,1),'Ydata',log_fw(:,2));  
    set(prw, 'Xdata', log_rw(:,1),'Ydata',log_rw(:,2));  
    set(pl4, 'Function',fn_path);
    set(pl5, 'Function',Vehicle_heading);
    set(pl6,'Xdata',cx1,'Ydata',cy1);
    
    drawnow

    axis([-5 40 -22.5 22.5]);
end
