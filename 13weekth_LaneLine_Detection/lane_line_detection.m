clear; clf; close all;

%% 0. image read and grayscale, crop
img_ori = imread('lanedetect.bmp'); % 3채널 uint8 (0~255)
img_gray = rgb2gray(img_ori); % 1채널 unit8 img_gray = double(img_gray); 
                              % uint8은 필터링 과정에 서 255를 넘어가 overflow 발생. double은 0-1사이 단계화.
                              % Lane Line은 Color가 필요없기 때문에 RGB를 gray로 바꾸어 사용한다.
                              % (0 ~ 255) -> (0.0 ~ 1.0)

figure(1); clf; imshow(uint8(img_gray));

col = length(img_gray(:,1)); 
row = length(img_gray(1,:));
img_gray = img_gray(col/2:col, 1:row);  % img_ori(672 * 1280 * 3) -> img_gray(227 * 1280)
figure(2); clf; imshow(uint8(img_gray));

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
subplot(211);imshow(uint8(G_X));
title('G_x');

subplot(212);
imshow(uint8(G_Y));title('G_y');

% Calcultae magnitude of edge
magnitude = sqrt((G_X.^2) + (G_Y.^2));  % Euclidean Distance = 대각선의 크기
figure(5); clf; imshow(uint8(magnitude)); 
title('Magnitude : sqrt(G_x^2 + G_y^2)');

%Calculate directions/orientations
theta = atan2(G_Y, G_X); theta = theta * 180/pi;
%Adjustment for negative directions, making all directions positive
col = length(img_gray(:,1)); row = length(img_gray(1,:)); 
for i=1:col
    for j=1:row
        if (theta(i,j)<0)
            theta(i,j)= 360 + theta(i,j);
        end 
    end
end

%% 2_2. quantization theta
qtheta = zeros(col, row);
% Adjusting directions to nearest 0, 45, 90, or 135 degree
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
BW = zeros (col, row);
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
BW = BW.*magnitude;