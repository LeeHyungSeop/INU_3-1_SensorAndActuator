%% GPS data를 3
clear; clc;

load('gps_wgs84.mat'); % gps data가 담겨져 있음 (인천대 바깥쪽 한바퀴 돌았던 정보)

len = length(gps(:,1)); % 

lat = gps(:,1); % latitude
lon = gps(:,2); % longitude
alt = gps(:,3); % altitude
state = gps(:,4); % GPS state : RTK, Float RTk, DGPS & no data
s_lat = gps(1,1); % start point GPS 시작점 s_lon = gps(1,2);
s_lon = gps(1,2);
s_alt = gps(1,3);
e_lat = gps(end,1); % end point GPS 종료점 e_lon = gps(end,2);
e_lon = gps(end,2);
e_alt = gps(end,3);

start_point = gps(1,:);
end_point = gps(len,:);

figure(1);
zlim([0 100]); hold on; grid on; title('GPS wgs84'); % wgs 좌표계를 사용하고 있다
xlabel('latitude'); ylabel('longitude'); zlabel('altitude'); % 경도, 위도, 고도 -> 3차원
plot3(start_point(1),start_point(2),start_point(3),'o', 'MarkerFaceColor', 'b'); % 시작점 blue
plot3(end_point(1),end_point(2),end_point(3),'o', 'MarkerFaceColor', 'r'); % 끝점 red

% 항상 RTK data만 받으면 좋지만, 그렇지 않은 경우가 있어서 경우를 나눈다
for t = 1:len
    if(gps(t,4) == 2) % 2에 해당하면 DGPS -> yellow
        plot3(gps(t,1), gps(t,2), gps(t,3),'yo','MarkerSize',5); 
    elseif(gps(t,4) == 5) % 5에 해당하면 Float RTK -> green
        plot3(gps(t,1), gps(t,2), gps(t,3),'go','MarkerSize',5);
    elseif(gps(t,4) == 4) % 4에 해당하면 RTK -> magenta
        plot3(gps(t,1), gps(t,2), gps(t,3),'mo','MarkerSize',5);
    elseif(gps(t,4) == 1) % 1에 해당하면 No Data -> Black
        plot3(gps(t,1), gps(t,2), gps(t,3),'ko','MarkerSize',5);
    end
end

%% 위성사진에 GPS data 그리기 wgs84 좌표계
figure(1);
webmap('World Imagery') 
wmmarker(s_lat,s_lon,'Color','r','IconScale',0.7) 
wmmarker(e_lat,e_lon,'Color','b','IconScale',0.7,'Alpha',0.7) 
wmline(lat,lon,'LineWidth',3,'FeatureName','coastline') 
wmzoom(17) % map 확대, 축소

%% GPS data를 3D로 그리기 (UTM 좌표계)
% utm 안에서도 종류가 있는데, wgs2utm
[utmX,utmY,utmzone,utmhemi] = wgs2utm(lat,lon,52,'S'); % 52 격자는 우리나라가 속해있는 격자 번호.
                                                       % 그 격자 안에서 UTM 좌표계로 변환할 것이다.

                                                       % utmzone, utmhemi는
                                                       % 이미 알고 있는 값이므로 사용하지
                                                       % 않고 utmX, utmY
figure(2); hold on; grid on; title('GPS UTM');
xlabel('x (m)');
ylabel('y (m)');
plot(utmX, utmY); 
plot(utmX(1,1),utmY(1,1),'o','MarkerFaceColor','b'); 
plot(utmX(len,1),utmY(len,1),'o','MarkerFaceColor','r');