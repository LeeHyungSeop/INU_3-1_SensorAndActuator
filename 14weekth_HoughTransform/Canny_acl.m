function output_img = Canny_acl(src_img ,filtersz, sigma, th_h, th_l)

%% gaussian filter
%Smoothing the input image by an Gaussian filter
g1=fspecial('gaussian', [filtersz,filtersz],sigma); % [5 * 5] filter size를 갖으며, 표준편차가 5인 gaussian filter를 만듦
img1=filter2(g1,src_img); %2D convolution

%% Calculating gradient with sobel mask
sobelMaskX=[-1,0,1;-2,0,2;-1,0,1]; 
sobelMaskY=[1,2,1;0,0,0;-1,-2,-1];

%Convolution by image by horizontal and vertical filter
G_X=conv2(img1,sobelMaskX,'same');   % 수평 방향(x) 미분 filter -> 수직 방향 edge 검출
G_Y=conv2(img1,sobelMaskY,'same');   % 수직 방향(y) 미분 filter -> 수평 방향 edge 검출 

%Calcultae magnitude of edge 
magnitude=sqrt((G_X.^2)+(G_Y.^2)); % Euclidean Distance = 대각선의 크기(edge의 세기)

%Calculate directions/orientations
theta=atan2(G_Y,G_X);   % arctan2를 이용하여 -pi ~ pi의 각도 추출
theta=theta*(180/pi);   % radian -> degree

%Adjustment for negative directions, making all directions positive
col=length(src_img(:,1));
row=length(src_img(1,:));

for i=1:col
   for j=1:row
        if (theta(i,j)<0)
            theta(i,j)= 360 + theta(i,j); % 0 <= theta <= 360
        end 
   end
end

%% quantization theta
qtheta=zeros(col,row);

% 앞서 구한 각도 theta를 group화
% 각 범위 [0, 360]를 4가지 구간으로 나누어 quantization 
% Adjusting directions to nearest 0, 0+45(45), 0+45+45(90), or 0+45+45+45(135) degree
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

%% Non-Maximum Supression

% 0~1 image를 0 or 1로 바꾸겠다 (지역적으로 최대값이 아닌 edge를 제거하는 과정.
% -> edge에 기여하지 않는 pixel 제거  == blurring을 통해 약한 edge 제거
% -> Sharp한 edge로 변경
BW=zeros(col,row);
for i=2:col-1
    for j=2:row-1
        % quantization theta=0인 애들끼리 비교. 가운데 pixel값이 max면? 1 아니면? 0
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

% BW는 0 or 1의 값을 갖게 되고, magnitude 행렬에 element-wise 곱을 하여 
% 0인 부분은 제거해줌. 1인 부분은 살려둠.
BW=BW.*magnitude;


%% Hysteresis Thresholding
T_max=th_h; T_min=th_l;       % threshold max, min
T_min=T_min * max(max(BW));   % 5% 미만, 초과인지 판단하기 위해 (미만? No edge)
T_max=T_max * max(max(BW));   % 5% 미만, 초과인지 판단하기 위해 (초과? Strong Edge)
edge_final = zeros(col, row); 

for i=1:col
    for j=1:row
        if (BW(i,j)<T_min)      % no edge라면? 0으로
            edge_final(i,j)=0;
        elseif(BW(i,j)>T_max)   % Strong edge라면? 1로
            edge_final(i,j)=1;
        % Weak edge - Using 8-connected components
        % 주변 8개가 모두 Strong Edge라면? 내가 뭐든지 간에 Strong Edge로 인정받을 수 있음
        elseif( BW(i+1,j)>T_max || BW(i-1,j)>T_max ...
            || BW(i,j+1)>T_max || BW(i,j-1)>T_max || BW(i-1,j-1)>T_max ...
            || BW(i-1,j+1)>T_max || BW(i+1,j+1)>T_max || BW(i+1,j-1)>T_max) ...
            edge_final(i,j)=1;
        end 
    end
end

output_img=uint8(edge_final.*255);  % output 이미지를 다시 [0~255]범위로 변환
end













