clear all; close all;


L = 2.7;		% L = wheelbase (m) for a 2017 avante(AD) model
                % https://www.carisyou.com/car/5462/Spec/53102

FTREAD = 1.545; % 윤거 전 : 앞 좌우 타이어 접지면의 중심간 수평거리                 
RTREAD = 1.559; % 윤거 후 : 뒤 좌우 타이어 접지면의 중심간 수평거리                 
%% steering angle = sine wave, 0.05 Hz
dt = 0.05;	% time step size (sec)
T = 10;		% sine wave period (T sec)
t = 0:dt:T-dt;	% time step vector (sec)
f = 1/T;		% frequency = 1/T

% 직진하며 진입
sta_amp = 00;	% Steering Angle Amplitude = 00 (deg)
sta1 = sta_amp * cosd(linspace(180,270, 25));  % steering angle step vector (deg)
% 좌회전
sta_amp = 70;	% Steering Angle Amplitude = 70 (deg)
sta2 = sta_amp * cosd(linspace(270,360, 50));  % steering angle step vector (deg)
% 우회전 하기 위해 회전 반경 넓히기
sta_amp = 45;	% Steering Angle Amplitude = 45 (deg)
sta3 = sta2(1, 50)*ones(1, 40) + sta_amp * cosd(linspace(270, 360, 40));  % steering angle step vector (deg)
% 우회전
sta_amp = 100;	% Steering Angle Amplitude = 100 (deg)
sta4 = sta3(1, 40)*ones(1, 60) + sta_amp * cosd(linspace(90, 180, 60));  % steering angle step vector (deg)
% 핸들 풀며 나가기
sta_amp = 15;	% Steering Angle Amplitude = 15 (deg)
sta5 = sta4(1, 60)*ones(1, 25) + sta_amp * cosd(linspace(90, 180, 25));  % steering angle step vector (deg)

sta = [sta1 sta2 sta3 sta4 sta5]

%% initial state
figure(1); clf; hold on; box on;
xlabel('x (m)'); ylabel('y (m)');
v_x= 0;		% vehicle initial speed to x-dir (0 m/sec)
v_y= 0;		% vehicle initial speed to y-dir (0 m/sec)
theta = 0;	% vehicle inital yaw angle (0 rad)
cmd_sta = deg2rad(0); % initial command of steering angle (0 rad)

v = 3; 	% vehicle speed to forward-dir = 3 m/s = 10.8 km/h 
lfd = [0, FTREAD/2]; % Coord of initial left front wheel center = [0, 윤거/2]
rfd = [0, -FTREAD/2]; % Coord of initial right front wheel center = [0, -윤거/2]
lfdx = cos(theta)*lfd(1) - sin(theta)*lfd(2); 
rfdx = cos(theta)*rfd(1) - sin(theta)*rfd(2);
lfdy = sin(theta)*lfd(1) + cos(theta)*lfd(2);
rfdy = sin(theta)*rfd(1) + cos(theta)*rfd(2);

lrd = [-L, RTREAD/2]; % Coord of initial left rear wheel center = [-L, 윤거/2]
rrd = [-L, -RTREAD/2];  % Coord of initial right rear wheel center = [-L, -윤거/2] 
lrdx = cos(theta)*lrd(1) - sin(theta)*lrd(2);
rrdx = cos(theta)*rrd(1) - sin(theta)*rrd(2);
lrdy = sin(theta)*lrd(1) + cos(theta)*lrd(2);
rrdy = sin(theta)*rrd(1) + cos(theta)*rrd(2);

[updatedLFx, updatedLFy, updatedLRx, updatedLRy, updatedRFx, updatedRFy, updatedRRx, updatedRRy,updatedVx, updatedVy] = ...
    my_veh_obj(theta, cmd_sta, lfdx, rfdx, lfdy, rfdy, lrdx, rrdx, lrdy, rrdy, v_x, v_y, L);

vehicle = fill(updatedVx,updatedVy,'w','EraseMode','normal'); 
Flfw = fill(updatedLFx,updatedLFy,'r','EraseMode','normal'); % left front wheel fill object
Frfw = fill(updatedRFx,updatedRFy,'r','EraseMode','normal'); % right front wheel fill object
Flrw = fill(updatedLRx,updatedLRy,'b','EraseMode','normal'); % left rear wheel fill object
Frrw = fill(updatedRRx,updatedRRy,'b','EraseMode','normal'); % right rear wheel fill object
lpd = plot([lfdx lrdx],[lfdy lrdy],'k','LineWidth',2);     % left driveshaft line object
rpd = plot([rfdx rrdx],[rfdy rrdy],'k','LineWidth',2);     % right driveshaft line object

track_lfx = ones(1,2); % left front wheel center (x,y) position 
track_rfx = ones(1,2); % right front wheel center (x,y) position 
track_lrx = ones(1,2); % left rear wheel center (x,y) position 
track_rrx = ones(1,2); % right rear wheel center (x,y) position 

% front wheel track line object
Ltrack_lfw = plot(track_lfx(:,1),track_lfx(:,2),'r','LineWidth',1); 
Ltrack_rfw = plot(track_rfx(:,1),track_rfx(:,2),'r','LineWidth',1); 

% rear wheel track line object
Ltrack_lrw = plot(track_lrx(:,1),track_lrx(:,2),'b:','LineWidth',1); 
Ltrack_rrw = plot(track_rrx(:,1),track_rrx(:,2),'b:','LineWidth',1); 

plot(0,0,'rx'); % plot front wheel initial (x,y) position (Origin of the coord)
%% simulation 
drawMap()  % 굴절코스 map drawing
for i = 1:1:length(t)
    cmd_sta = deg2rad(sta(i)) - theta;
    v_x = v_x + v * cos( theta + cmd_sta ) * dt;
    v_y = v_y + v * sin( theta + cmd_sta ) * dt;
    theta = theta + v * tan(cmd_sta)/L * dt;
   
    lfdx = cos(theta)*lfd(1) - sin(theta)*lfd(2); 
    lfdy = sin(theta)*lfd(1) + cos(theta)*lfd(2);
    rfdx = cos(theta)*rfd(1) - sin(theta)*rfd(2);
    rfdy = sin(theta)*rfd(1) + cos(theta)*rfd(2);

    lrdx = cos(theta)*lrd(1) - sin(theta)*lrd(2);
    lrdy = sin(theta)*lrd(1) + cos(theta)*lrd(2);
    rrdx = cos(theta)*rrd(1) - sin(theta)*rrd(2);
    rrdy = sin(theta)*rrd(1) + cos(theta)*rrd(2);

    [updatedLFx, updatedLFy, updatedLRx, updatedLRy, updatedRFx, updatedRFy, updatedRRx, updatedRRy,updatedVx, updatedVy] ...
    = my_veh_obj(theta, cmd_sta, lfdx, rfdx, lfdy, rfdy, lrdx, rrdx, lrdy, rrdy, v_x, v_y, L);

    set(vehicle, 'Xdata', updatedVx, 'Ydata', updatedVy); 
    set(Flfw, 'Xdata', updatedLFx,'Ydata', updatedLFy); 
    set(Flrw, 'Xdata', updatedLRx,'Ydata', updatedLRy);
    set(Frfw, 'Xdata', updatedRFx,'Ydata', updatedRFy); 
    set(Frrw, 'Xdata', updatedRRx,'Ydata', updatedRRy);
    
    log_lfw(i,:) = [lfdx+v_x lfdy+v_y];
    log_rfw(i,:) = [rfdx+v_x rfdy+v_y];
    log_lrw(i,:) = [lrdx+v_x lrdy+v_y];
    log_rrw(i,:) = [rrdx+v_x rrdy+v_y];
    
    set(lpd, 'Xdata', [lfdx+v_x lrdx+v_x],'Ydata',[lfdy+v_y lrdy+v_y]); % Update driveshaft
    set(rpd, 'Xdata', [rfdx+v_x rrdx+v_x],'Ydata',[rfdy+v_y rrdy+v_y]); % Update driveshaft
    set(Ltrack_lfw, 'Xdata', log_lfw(:,1),'Ydata',log_lfw(:,2));  % Update track of lfw
    set(Ltrack_rfw, 'Xdata', log_rfw(:,1),'Ydata',log_rfw(:,2));  % Update track of rfw
    set(Ltrack_lrw, 'Xdata', log_lrw(:,1),'Ydata',log_lrw(:,2));  % Update track of lrw
    set(Ltrack_rrw, 'Xdata', log_rrw(:,1),'Ydata',log_rrw(:,2));  % Update track of rrw

    title(strcat('steering angle : ',num2str(rad2deg(cmd_sta)),'(deg)' ));
    axis equal; grid on;
    axis([v_x-11 v_x+9 v_y-10 v_y+10])
    drawnow limitrate
    
    F(i) = getframe(gcf);
end

% Redraw whole simulation
axis([-5 25 -5 20])
title('result')
drawnow limitrate
F(i+1) = getframe(gcf);