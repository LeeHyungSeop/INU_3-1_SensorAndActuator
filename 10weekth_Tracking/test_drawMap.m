function test_drawMap()

% 상단 경계
upper_line_x = [0, 6-1.5, 6, 6, 16.7];
upper_line_y = [4.7, 4.7, 4.7+1.5, 19.7, 19.7] - 1*ones(1,5);
% 하단 경계
lower_line_x = [0, 10.7, 10.7, 10.7+1.5, 16.7];
lower_line_y = [0, 0, 15-1.5, 15, 15]- 1*ones(1,5);
% 모퉁이 그릴 곳에 라인이 그려지니까 지우기 위해
erase1_x = [6-1.5, 6];
erase1_y = [4.7, 4.7+1.5] - 1*ones(1,2);
erase2_x = [10.7, 10.7+1.5];
erase2_y = [15-1.5, 15] - 1*ones(1,2);

hold on;
plot(upper_line_x, upper_line_y, '-', 'Color', [1 0.5 0], 'LineWidth', 3);
plot(lower_line_x, lower_line_y, '-', 'Color', [1 0.5 0], 'LineWidth', 3);
plot(erase1_x, erase1_y, '-', 'Color', [1 1 1], 'LineWidth', 3);
plot(erase2_x, erase2_y, '-', 'Color', [1 1 1], 'LineWidth', 3);

% 중심 좌표와 반지름
r = 1.5;
c1_x = 6 - r;
c1_y = 4.7 + r;
c2_x = 10.7 + r;
c2_y = 15 - r;

% 시작각과 종료각 (모퉁이 그릴 때, (원 둘레 * (1/4))만 그리기 위해)
theta_start_1 = pi + pi/2;
theta_end_1 = 2*pi;
theta_start_2 = pi/2;
theta_end_2 = pi;

% 원의 좌표 계산
theta_1 = linspace(theta_start_1, theta_end_1, 50); 
x1 = r * cos(theta_1) + c1_x; % 첫 번째 원의 x좌표
y1 = r * sin(theta_1) + c1_y - 1; % 첫 번째 원의 y좌표
theta_2 = linspace(theta_start_2, theta_end_2, 50); 
x2 = r * cos(theta_2) + c2_x; % 두 번째 원의 x좌표
y2 = r * sin(theta_2) + c2_y - 1; % 두 번째 원의 y좌표

% 그래프 그리기
hold on;
plot(x1, y1, 'Color', [1 0.5 0], 'LineWidth', 3); % 첫 번째 원 그리기
plot(x2, y2, 'Color', [1 0.5 0], 'LineWidth', 3); % 두 번째 원 그리기
axis equal; % x, y축 비율 동일하게 설정
