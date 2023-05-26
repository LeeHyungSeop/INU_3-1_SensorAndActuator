function [updatedFx, updatedFy, updatedRx, updatedRy, updatedVx, updatedVy] = ...
    veh_obj(theta, cmd_sta, fdx, fdy, rdx, rdy, v_x, v_y,L)


Loa = 4.57; % Overall length (4.57 m)
Woa = 1.8;  % Overall width (1.8 m)
Lwh = 0.7;  % wheel length (0.7 m)
Wwh = 0.42; % wheel width (0.42 m)
shape_x = [-Lwh/2 Lwh/2 Lwh/2 -Lwh/2]; 
shape_y = [Wwh/2 Wwh/2 -Wwh/2 -Wwh/2]; 
shape_vx = [-L-(Loa-L)/2 (Loa-L)/2 (Loa-L)/2 -L-(Loa-L)/2];
shape_vy = [Woa/2 Woa/2 -Woa/2 -Woa/2];
updatedFx = [shape_x(1) * cos(theta+cmd_sta) - shape_y(1) * sin(theta+cmd_sta) + fdx + v_x
        shape_x(2) * cos(theta+cmd_sta) - shape_y(2) * sin(theta+cmd_sta) + fdx + v_x
        shape_x(3) * cos(theta+cmd_sta) - shape_y(3) * sin(theta+cmd_sta) + fdx + v_x
        shape_x(4) * cos(theta+cmd_sta) - shape_y(4) * sin(theta+cmd_sta) + fdx + v_x];
updatedFy = [shape_x(1) * sin(theta+cmd_sta) + shape_y(1) * cos(theta+cmd_sta) + fdy + v_y
        shape_x(2) * sin(theta+cmd_sta) + shape_y(2) * cos(theta+cmd_sta) + fdy + v_y
        shape_x(3) * sin(theta+cmd_sta) + shape_y(3) * cos(theta+cmd_sta) + fdy + v_y
        shape_x(4) * sin(theta+cmd_sta) + shape_y(4) * cos(theta+cmd_sta) + fdy + v_y]; 
updatedRx = [shape_x(1) * cos(theta) - shape_y(1) * sin(theta) + rdx + v_x
        shape_x(2) * cos(theta) - shape_y(2) * sin(theta) + rdx + v_x
        shape_x(3) * cos(theta) - shape_y(3) * sin(theta) + rdx + v_x
        shape_x(4) * cos(theta) - shape_y(4) * sin(theta) + rdx + v_x];
updatedRy = [shape_x(1) * sin(theta) + shape_y(1) * cos(theta) + rdy + v_y
        shape_x(2) * sin(theta) + shape_y(2) * cos(theta) + rdy + v_y
        shape_x(3) * sin(theta) + shape_y(3) * cos(theta) + rdy + v_y
        shape_x(4) * sin(theta) + shape_y(4) * cos(theta) + rdy + v_y];
updatedVx = [shape_vx(1) * cos(theta) - shape_vy(1) * sin(theta) + v_x
        shape_vx(2) * cos(theta) - shape_vy(2) * sin(theta) + v_x
        shape_vx(3) * cos(theta) - shape_vy(3) * sin(theta) + v_x
        shape_vx(4) * cos(theta) - shape_vy(4) * sin(theta) + v_x];
updatedVy = [shape_vx(1) * sin(theta) + shape_vy(1) * cos(theta) + v_y
        shape_vx(2) * sin(theta) + shape_vy(2) * cos(theta) + v_y
        shape_vx(3) * sin(theta) + shape_vy(3) * cos(theta) + v_y
        shape_vx(4) * sin(theta) + shape_vy(4) * cos(theta) + v_y];
end
