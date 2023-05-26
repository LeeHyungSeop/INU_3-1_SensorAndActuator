function [updatedLFx, updatedLFy, updatedLRx, updatedLRy, updatedRFx, updatedRFy, updatedRRx, updatedRRy,updatedVx, updatedVy] = ...
    my_veh_obj(theta, cmd_sta, lfdx, rfdx, lfdy, rfdy, lrdx, rrdx, lrdy, rrdy, v_x, v_y, L)


Loa = 4.57; % Overall length : 전장(4.570 m)
Woa = 1.8;  % Overall width : 전폭(1.8 m)
Lwh = 0.7;  % wheel length (0.7 m)
Wwh = 0.42; % wheel width (0.42 m)

shape_x = [-Lwh/2 Lwh/2 Lwh/2 -Lwh/2]; 
shape_y = [Wwh/2 Wwh/2 -Wwh/2 -Wwh/2]; 
shape_vx = [(-L-(Loa-L)/2) ((Loa-L)/2) ((Loa-L)/2) (-L-(Loa-L)/2)];
shape_vy = [Woa/2 Woa/2 -Woa/2 -Woa/2];

% 왼쪽 앞, 뒤 바퀴에 대한 CCW
updatedLFx = [shape_x(1) * cos(theta+cmd_sta) - shape_y(1) * sin(theta+cmd_sta) + lfdx + v_x
        shape_x(2) * cos(theta+cmd_sta) - shape_y(2) * sin(theta+cmd_sta) + lfdx + v_x
        shape_x(3) * cos(theta+cmd_sta) - shape_y(3) * sin(theta+cmd_sta) + lfdx + v_x
        shape_x(4) * cos(theta+cmd_sta) - shape_y(4) * sin(theta+cmd_sta) + lfdx + v_x];
updatedLFy = [shape_x(1) * sin(theta+cmd_sta) + shape_y(1) * cos(theta+cmd_sta) + lfdy + v_y
        shape_x(2) * sin(theta+cmd_sta) + shape_y(2) * cos(theta+cmd_sta) + lfdy + v_y
        shape_x(3) * sin(theta+cmd_sta) + shape_y(3) * cos(theta+cmd_sta) + lfdy + v_y
        shape_x(4) * sin(theta+cmd_sta) + shape_y(4) * cos(theta+cmd_sta) + lfdy + v_y]; 
updatedLRx = [shape_x(1) * cos(theta) - shape_y(1) * sin(theta) + lrdx + v_x
        shape_x(2) * cos(theta) - shape_y(2) * sin(theta) + lrdx + v_x
        shape_x(3) * cos(theta) - shape_y(3) * sin(theta) + lrdx + v_x
        shape_x(4) * cos(theta) - shape_y(4) * sin(theta) + lrdx + v_x];
updatedLRy = [shape_x(1) * sin(theta) + shape_y(1) * cos(theta) + lrdy + v_y
        shape_x(2) * sin(theta) + shape_y(2) * cos(theta) + lrdy + v_y
        shape_x(3) * sin(theta) + shape_y(3) * cos(theta) + lrdy + v_y
        shape_x(4) * sin(theta) + shape_y(4) * cos(theta) + lrdy + v_y];
% 왼쪽 앞, 뒤 바퀴에 대한 CCW
updatedRFx = [shape_x(1) * cos(theta+cmd_sta) - shape_y(1) * sin(theta+cmd_sta) + rfdx + v_x
        shape_x(2) * cos(theta+cmd_sta) - shape_y(2) * sin(theta+cmd_sta) + rfdx + v_x
        shape_x(3) * cos(theta+cmd_sta) - shape_y(3) * sin(theta+cmd_sta) + rfdx + v_x
        shape_x(4) * cos(theta+cmd_sta) - shape_y(4) * sin(theta+cmd_sta) + rfdx + v_x];
updatedRFy = [shape_x(1) * sin(theta+cmd_sta) + shape_y(1) * cos(theta+cmd_sta) + rfdy + v_y
        shape_x(2) * sin(theta+cmd_sta) + shape_y(2) * cos(theta+cmd_sta) + rfdy + v_y
        shape_x(3) * sin(theta+cmd_sta) + shape_y(3) * cos(theta+cmd_sta) + rfdy + v_y
        shape_x(4) * sin(theta+cmd_sta) + shape_y(4) * cos(theta+cmd_sta) + rfdy + v_y]; 
updatedRRx = [shape_x(1) * cos(theta) - shape_y(1) * sin(theta) + rrdx + v_x
        shape_x(2) * cos(theta) - shape_y(2) * sin(theta) + rrdx + v_x
        shape_x(3) * cos(theta) - shape_y(3) * sin(theta) + rrdx + v_x
        shape_x(4) * cos(theta) - shape_y(4) * sin(theta) + rrdx + v_x];
updatedRRy = [shape_x(1) * sin(theta) + shape_y(1) * cos(theta) + rrdy + v_y
        shape_x(2) * sin(theta) + shape_y(2) * cos(theta) + rrdy + v_y
        shape_x(3) * sin(theta) + shape_y(3) * cos(theta) + rrdy + v_y
        shape_x(4) * sin(theta) + shape_y(4) * cos(theta) + rrdy + v_y];
% 왼쪽 앞, 뒤 바퀴에 대한 CCW
updatedVx = [shape_vx(1) * cos(theta) - shape_vy(1) * sin(theta) + v_x
        shape_vx(2) * cos(theta) - shape_vy(2) * sin(theta) + v_x
        shape_vx(3) * cos(theta) - shape_vy(3) * sin(theta) + v_x
        shape_vx(4) * cos(theta) - shape_vy(4) * sin(theta) + v_x];
updatedVy = [shape_vx(1) * sin(theta) + shape_vy(1) * cos(theta) + v_y
        shape_vx(2) * sin(theta) + shape_vy(2) * cos(theta) + v_y
        shape_vx(3) * sin(theta) + shape_vy(3) * cos(theta) + v_y
        shape_vx(4) * sin(theta) + shape_vy(4) * cos(theta) + v_y];
end
