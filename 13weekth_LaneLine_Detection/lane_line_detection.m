clear; clf; close all;

%% 0. image read and grayscale, crop
img_ori = imread('lanedetect.bmp'); % 3채널 uint8 (0~255)
img_gray = rgb2gray(img_ori); % 1채널 unit8 img_gray = double(img_gray); 
                              % uint8은 필터링 과정에 서 255를 넘어가 overflow 발생. double은 0-1사이 단계화.
                              % Lane Line은 Color가 필요없기 때문에 RGB를 gray로 바꾸어 사용한다.
                              % (0 ~ 255) -> (0.0 ~ 1.0)

figure(1); clf; imshow(uint8(img_gray));
title("Original Image");

col = length(img_gray(:,1)); 
row = length(img_gray(1,:));
img_gray = img_gray(col/2:col, 1:row);  % img_ori(672 * 1280 * 3) -> img_gray(227 * 1280)
figure(2); clf; imshow(uint8(img_gray));
title("Cropped Image");
%% 1. gaussian filter

% 정규분포    : Gaussian Distribution.            (mu=?, sigma=?)
% 표준정규분포 : Normalized Gaussian Distribution. (mu=0, sigma=1)

% 정규분포로 나타내겠다.
g1 = fspecial('gaussian', [5, 5], 5);       % fspecial('gaussian', (size of filter=[5, 5]), (sigma=[5]))
g2 = fspecial('gaussian', [5, 5], 10);      % fspecial('gaussian', (size of filter=[5, 5]), (sigma=[10]))
g3 = fspecial('gaussian', [11, 11], 5);     % fspecial('gaussian', (size of filter=[11, 11]), (sigma=[5]))
                                            % 2D로 나타내기 위해서 pixel 개수 [5, 5]와
                                            % sigma를 통해 spread를 정함
                                            
                                            % 왜 filter를 만드는가?
                                            % 주변에 있는 8개의 information을 축소,
                                            % 가운데 있는 대표값을 최대화.

% 2D Convolution 연산
img1 = filter2(g1,img_gray, 'same');       
img2 = filter2(g2,img_gray, 'same'); 
img3 = filter2(g3,img_gray, 'same');


% sigma값이 클수록 blurring현상이 커짐.(sigma값이 크면 spread가 크기 때문에 특징을 잘 끄집어내지 못하여 흐릿하게 보임)
figure(3); clf;
subplot(2,2,1)
imshow(uint8(img_gray)); title('Original Image', 'FontSize', 10); subplot(2,2,2)

imshow(uint8(img1));
title('Gaussian filtered Image, size = 5x5, \sigma = 5', 'FontSize', 10); subplot(2,2,3)
imshow(uint8(img2));
title('Gaussian filtered Image, size = 5x5, \sigma = 10', 'FontSize', 10); subplot(2,2,4)
imshow(uint8(img3));
title('Gaussian filtered Image, size = 11x11, \sigma = 5', 'FontSize', 10);

%% 2_1. Calculating gradient with sobel mask
sobelMaskX = [-1, 0, 1; -2, 0, 2; -1, 0, 1];    % 수평 방향(x) 미분 filter -> 수직 방향 edge 검출
sobelMaskY = [1, 2, 1; 0, 0, 0; -1, -2, -1];    % 수직 방향(y) 미분 filter -> 수평 방향 edge 검출 
% Convolution with horizontal and vertical filter
G_X = conv2(img1, sobelMaskX, 'same');
G_Y = conv2(img1, sobelMaskY, 'same');

figure(4); 
clf; 
subplot(211);
imshow(uint8(G_X));
title('G_x');

subplot(212);
imshow(uint8(G_Y));
title('G_y');

% Calcultae magnitude of edge
magnitude = sqrt((G_X.^2) + (G_Y.^2));  % Euclidean Distance = 대각선의 크기 (피타고라스)
figure(5); clf; imshow(uint8(magnitude)); 
title('Magnitude : sqrt(G_x^2 + G_y^2)');

%Calculate directions/orientations
theta = atan2(G_Y, G_X);
theta = theta * 180/pi;
%Adjustment for negative directions, making all directions positive
col = length(img_gray(:,1)); 
row = length(img_gray(1,:)); 
for i=1:col
    for j=1:row
        if (theta(i,j)<0)
            theta(i,j)= 360 + theta(i,j); % range를 [0, 360]으로
        end 
    end
end

%% 2_2. quantization theta

% 앞서 구한 각도 theta를 group화
% 각 범위 [0, 360]를 4가지 구간으로 나누어 quantization 
% Adjusting directions to nearest 0, 0+45(45), 0+45+45(90), or 0+45+45+45(135) degree
qtheta = zeros(col, row);

% pdf slide 21 참고
for i = 1 :col 
    for j = 1 : row 
        if (((theta(i,j) >= 0) && (theta(i, j) < 22.5 )) || ... 
            ((theta(i,j) >= 157.5) && (theta(i, j) < 202.5)) || ... 
            ((theta(i,j) >= 337.5) && (theta(i, j) <= 360 )))
            qtheta(i, j) = 1; % degree group 0
        elseif (((theta(i, j) >= 22.5) && (theta(i, j) < 67.5 )) || ... 
                ((theta(i, j) >= 202.5)&& (theta(i, j) < 247.5)))
                qtheta(i, j) = 1; % degree group 1
        elseif (((theta(i, j) >= 67.5 && theta(i, j) < 112.5)) || ...
                ((theta(i, j) >= 247.5 && theta(i, j) < 292.5))) 
                qtheta(i, j) = 2; % degree group 2
        elseif (((theta(i, j) >= 112.5 && theta(i, j) < 157.5)) || ... 
                ((theta(i, j) >= 292.5 && theta(i, j) < 337.5)))
                 qtheta(i, j) = 3; % degree group 3 
        end 
    end
end
%% 3. Non-Maximum Supression

% 0~1 image를 0 or 1로 바꾸겠다 (지역적으로 최대값이 아닌 edge를 제거하는 과정. -> edge에 기여하지 않는 pixel 제거 -> Sharp한 edge로 변경)
BW = zeros (col, row); % 모두 0으로 만듦

for i=2:col-1
    for j=2:row-1
        if (qtheta(i,j)==0)
            BW(i,j) = (magnitude(i,j) == max([magnitude(i,j), ... 
            magnitude(i,j+1), magnitude(i,j-1)]));
        elseif (qtheta(i,j) == 1)
            BW(i,j) = (magnitude(i,j) == max([magnitude(i,j), ...
            magnitude(i+1,j-1), magnitude(i-1,j+1)]));
        elseif (qtheta(i,j) == 2)
            BW(i,j) = (magnitude(i,j) == max([magnitude(i,j), ...
            magnitude(i+1,j), magnitude(i-1,j)])); 
        elseif (qtheta(i,j) == 3)
            BW(i,j) = (magnitude(i,j) == max([magnitude(i,j), ... 
            magnitude(i+1,j+1), magnitude(i-1,j-1)]));
        end 
    end
end

BW_0 = BW;
imshow(uint8((BW_0)));

% BW는 0 or 1의 값을 갖게 되고, magnitude 행렬에 element-wise 곱을 하여 
% 0인 부분은 제거해줌. 1인 부분은 살려둠.
BW = BW.*magnitude;


%% Hysteresis Thresholding

T_min = 0.1; T_max = 0.2; % threshold max, min
% max(max(BW))   : 전체 pixel값 중 최대값
T_min = T_min * max(max(BW)); % 10% 미만, 초과인지 판단하기 위해 (미만? No edge, )
T_max = T_max * max(max(BW)); % 20% 미만, 초과인지 판단하기 위해 (미만인데 10% 초과? Weak Edge, 20%초과? Strong Edge)
                              % 정리) < 10% ? No edge / 10 < < 20 ? Weak edge / 20 < ? Strong edge 
edge_final = zeros (col, row); 
for i=1 :col
    for j = 1 : row
        if (BW(i, j) < T_min) % no edge라면? 0으로
            edge_final(i, j) = 0;
        elseif (BW(i, j) > T_max) % Strong edge라면? 1로
            edge_final(i, j) = 1;

        % Weak edge - Using 8-connected components
        % 주변 8개가 모두 Strong Edge라면? 내가 뭐든지 간에 Strong Edge로 인정받을 수 있음
        elseif (BW(i+1,j)>T_max || BW(i-1,j)>T_max || BW(i,j+1)>T_max ...
                || BW(i,j-1)>T_max || BW(i-1, j-1)>T_max || BW(i-1, j+1)>T_max ... 
                || BW(i+1, j+1)>T_max || BW(i+1, j-1)>T_max)
                edge_final(i,j) = 1;
                edge_final(i, j) = 1;
        end 
    end
end
img_canny = uint8(edge_final.*255); output_img = img_canny;
figure(6); clf; imshow(output_img);
title("Canny Edge Detection Algorithm 직접 구현 결과");

%% Matlab edge('Canny') -> 위에 했던 모든 알고리즘을 한꺼번에 구현해줌...
img_ori = imread('lanedetect.bmp');
img_gray = rgb2gray(img_ori);
img_gray = img_gray(length(img_gray(:,1))/2:end, 1:end); 
img_edge = edge(img_gray, 'Canny', [0.1 0.2], 0.5); 
                               % [T_min T_max], Sigma of Gaussian filter = 0.5
% edge() : Matlab 지원 함수

figure(); imshow(img_edge);
title("Matlab edge('Canny') function 결과");