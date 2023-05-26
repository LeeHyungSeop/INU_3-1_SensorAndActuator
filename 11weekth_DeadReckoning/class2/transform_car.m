function [car_re] = transform_car(x,y,theta)

persistent car_data car_x car_y car_z car
persistent firstRun

if isempty(firstRun)
    car_data=load('car.mat');
    car_x=car_data.car_X;
    car_y=car_data.car_Y;
    car_z=car_data.car_Z;
    car=[car_x;car_y;car_z;ones(1,length(car_x))];
    firstRun=1;
end


R_car_yaw=[cos(theta) -sin(theta) 0 0;
           sin(theta) cos(theta) 0 0;
           0 0 1 0;
           0 0 0 1];         
       

L_car=[1 0 0 x; 
       0 1 0 y; 
       0 0 1 0; 
       0 0 0 1];
      
car_re= L_car * R_car_yaw  *car;

end

