function [x1,y1,th1] = dead_reckoning(x,y,th,vk,wk,dt)
%% Runge-Kutta method
x1 = x + vk*dt*cos(th + (wk*dt/2));
y1 = y + vk*dt*sin(th + (wk*dt/2));
th1 = th + wk*dt;
end