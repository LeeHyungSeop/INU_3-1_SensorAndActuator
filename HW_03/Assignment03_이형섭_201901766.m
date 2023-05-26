clc; clear;

%% Ros Bag file 기록 및 분석 API
% https://kr.mathworks.com/help/ros/ros-bag-file-analysis.html?s_tid=CRUX_lftnav

%% 1. Load bag file : https://kr.mathworks.com/help/ros/ref/rosbag.html

% rosbag() : 경로에 있는 rosbag의 모든 Message index를 포함하고 있으면서 indexing 가능한 BagSelection 객체인 bag을 만듭니다.
% 데이터에 액세스하려면 readMessages 함수 또는 timeseries 함수를 호출하여 관련 데이터를 추출합니다.
bag_select = rosbag('/Users/ihyeongseob/Desktop/2023-05-04-16-44-38.bag');  
rosbag info '/Users/ihyeongseob/Desktop/2023-05-04-16-44-38.bag'; % bag file에 대한 information 확인하기

% rosbag info를 통해 Topcis를 확인 한 후, Topic 기준으로 filtering
gps_select = select(bag_select,'Topic','/fix'); % 
lidar_select = select(bag_select,'Topic','/ouster/points');

%% 2. rosbag에서 structure로 Message 읽기 : https://kr.mathworks.com/help/ros/ref/readmessages.html

gps_msgs = readMessages(gps_select,'DataFormat','struct'); % Message를 'structure'로 읽음
latitude = cellfun(@(m) double(m.Latitude), gps_msgs);  
longitude = cellfun(@(m) double(m.Longitude), gps_msgs);
altitude = cellfun(@(m) double(m.Altitude), gps_msgs);

lidar_msgs = readMessages(lidar_select,'DataFormat','struct'); % Message를 'structure'로 읽음

%% 3. GPS Data Visualization

gps_msgs_len = length(gps_msgs(:,1)); % gps_msgs 총 길이

start_lat = latitude(1); % GPS Start Point
start_lon = longitude(1);
start_alt = altitude(1);
end_lat = latitude(gps_msgs_len); % GPS End Point
end_lon = longitude(gps_msgs_len);
end_alt = altitude(gps_msgs_len);

start_point = [start_lat, start_lon, start_alt]; % 시작점의 (lat, long, alt) 정보 저장
end_point = [end_lat, end_lon, end_alt]; % 끝점의 (lat, long, alt) 정보 저장

figure(1);
hold on; grid on; title('GPS Data Visualization'); 
xlabel('Latitude(경도)'); ylabel('Longitude(위도)'); zlabel('Altitude(고도)');      % 경도, 위도, 고도 -> 3차원
plot3(latitude, longitude, altitude);
plot3(start_point(1),start_point(2),start_point(3),'o', 'MarkerFaceColor', 'b'); % Start Point Color : blue
plot3(end_point(1),end_point(2),end_point(3),'o', 'MarkerFaceColor', 'r');       % End Point Color : red

%% 4. 위성사진에 GPS Data(= 주행 경로) Visualization

webmap('World Imagery') 
wmmarker(start_lat, start_lon,'Color','b','IconScale', 1)              % Start Point Color : blue
wmmarker(end_lat,end_lon,'Color','r','IconScale', 1,'Alpha',0.7)       % End Point Color : red
wmline(latitude, longitude,'LineWidth', 5, 'FeatureName','coastline') 
wmzoom(17) % map 확대, 축소

%% 5. GPS data를 UTM 좌표로 바꾸기

[gps_utm_x, gps_utm_y, utmzone, utmhemi] = wgs2utm(latitude,longitude,52,'S'); 

% Plot UTM coordinates
figure(2);
plot(gps_utm_x, gps_utm_y);
grid on;
title('UTM Coordinates');

%% 6. data가 너무 많아서 lidar_data를 1/2로 줄이기

lidar_msgs_len = length(lidar_msgs);

lidar_datas = cell(lidar_msgs_len, 1);  % lidar data 추출
for i = 1:2:length(lidar_msgs)          % 1/2로 줄이기 위해 step size를 2
    lidar_datas{i} = rosReadXYZ(lidar_msgs{i});     % lidar의 X, Y, Z data 추출

    if mod(i, 300) == 0
        disp(i)                         % 진행 상황 확인
    end
end

lidar_data_combined = cat(1, lidar_datas{:});  % 추출한 lidar data를 lidar_data_combined에 저장


%% 7. lidar data개수를 gps data 개수와 똑같이 맞춰주기. 
% GPS data에 lidar에서 획득한 (x, y, z)를 더해주기 위함  

resize_scale = 5;
resize_var = round(gps_msgs_len/lidar_msgs_len, 2); % lidar data 개수를 gps dat 개수와 맞춰주기                                              

% lidar data 단위가 GPS data 단위보다 한참 작으므로 scaling.
resize_gps_utm_x = gps_utm_x(1 : resize_var : end) * resize_scale; 
resize_gps_utm_y = gps_utm_y(1 : resize_var : end) * resize_scale;

%% 8. draw lidar map & UTM GPS
figure(3)
hold on; grid on; axis('equal');
title('Lidar Map & GPS Trace');

% UTM 변환된 GPS data 그리기
plot(resize_gps_utm_x, resize_gps_utm_y, 'o', 'MarkerFaceColor','k', 'MarkerEdgeColor', 'k'); 
% UTM 좌표에 lidar data를 더해 lidar data로 map 그리기
for i = 1 : length(resize_gps_utm_x)
    plot(lidar_data_combined(i, 1) + resize_gps_utm_x(i),...
          lidar_data_combined(i, 2) + resize_gps_utm_y(i),...
          '.', 'MarkerSize', 5);
    i
end
% 출발점, 도착점 그리기
plot(resize_gps_utm_x(1,1), resize_gps_utm_y(1,1),'o','MarkerFaceColor','b', 'MarkerEdgeColor', 'b'); 
plot(resize_gps_utm_x(end,1), resize_gps_utm_y(end,1), 'o','MarkerFaceColor','r', 'MarkerEdgeColor', 'r'); 