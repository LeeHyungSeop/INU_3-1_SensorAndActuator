clear all; clc;
filename = 'challenge.mp4';

% challenge.mp4
% solidWhiteRight.mp4
% solidYellowLeft.mp4

VideoSource = vision.VideoFileReader(filename, 'VideoOutputDataType', 'double');
VideoOut = vision.VideoPlayer('Name', 'Output');

% Canny Edge Detection 과정에서 사용할 Gaussian Mask에 필요한 변수들 선언
filtersz = 3;   % filter size = 3
sigma = 3;      % \sigma = 3
th_h = 0.05;    % Hysteresis Thresholding을 위한 threshold max
th_l = 0.05;    % Hysteresis Thresholding을 위한 threshold min
ang_l = 75 ;    % lane line angle for Lane selection

while ~isDone(VideoSource)
    img = step(VideoSource);        % 현재 Video에서 img 추출
    img_ori = imresize(img, 1);     % 현재 img의 scale 조정 (https://kr.mathworks.com/help/matlab/ref/imresize.html)
    col = length(img_ori(:,1));     % 한 column에 있는 data 개수
    row = length(img_ori(1,:));     % 한 row에 있는 data 개수
    img_output = imcrop(img_ori,[1 col*(4/5) row col]); % img의 하위 2/5를 남겨놓고 상위 3/5 자름 (https://kr.mathworks.com/help/images/ref/imcrop.html)

    img_gray = rgb2gray(img_output);  % 1채널 unit8 i
    img_gray = double(img_gray);      % uint8은 필터링 과정에 서 255를 넘어가 overflow 발생. 
                                      % double은 0-1사이 단계화.
                                      % Lane Line은 Color가 필요없기 때문에 RGB를 gray로 바꾸어 사용한다.
                                      % (0 ~ 255) -> (0.0 ~ 1.0)

    %% canny edge detection
    image_canny = Canny_acl(img_gray, filtersz, sigma, th_h, th_l);

    %% Hough line transform
    % x-y domain(x, y) --> Hough domain 변환(Theta, Rho)
    [H,T,R] = hough(image_canny);  % [length(r) * length(theta), 180(-90 : 1 : 89)*1, sqrt(pixel_x^2 + pixel_y^2)*1]
    P = houghpeaks(H, 35,'threshold',ceil(0.3*max(H(:)))); 
        % H : length(r) * length(theta)
        % 25 : Max number of peaks : H에서 return된 점의 개수가 가장 많은 순서 25개
        % ceil() : Minimum value to be considered as a peak
    [lines] = houghlines(image_canny,T,R,P,'FillGap',10,'MinLength', 8);
        % FillGap : If the distance between two line segments is less than
        % this value, merge the line segments into a single line segment
        % MinLength : Minimum line length

    %% Lane selection
    c1 = []; c2 = []; l = [];
    for k = 1:length(lines)
        % 실제 차선이 존재하는 각도 범위 내에 있으면 차선으로 인정.
        if(lines(k).theta < 75 && lines(k).theta > -75)  % if(lines(k).theta < 60 && lines(k).theta > -60)  
            c1 = [c1; [lines(k).point1 2]];     % 시작점
            c2 = [c2; [lines(k).point2 2]];     % 끝점
            l = [l;lines(k).point1 lines(k).point2]; %
        end
    end
    img_line = insertShape(img_output, 'Line', l,'Color','green','LineWidth',3); % edge line 그리기 
    img_line = insertShape(img_line, 'Circle', c1); % 시작점 그리기
    img_line = insertShape(img_line, 'Circle', c2); % 끝점 그리기
    step(VideoOut, img_line);
end
release(VideoOut);
release(VideoSource);