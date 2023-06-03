%% Canny Edge Detection
% 2번 미분하여, zero crossing point를 찾아 Edge를 Detection.

% 미분 : delta x가 무한히 0이 될 떄의 y 변화량(=0), 차분 : 변화량
sig = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
d_sig = diff(sig);      % 1번 차분
dd_sig = diff(d_sig);   % 2번 차분(-> zero crossing point를 찾을 수 있음 == Edge Detection할 수 있음(어디서부터 line이 시작되었고 끝났는지))

figure(3);
subplot(3, 1, 1); plot(sig);
subplot(3, 1, 2); plot(d_sig);
subplot(3, 1, 3); plot(dd_sig);
