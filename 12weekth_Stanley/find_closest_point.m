function idx = find_closest_point(x_v,y_v,path_x,path_y)

for i = 1:length(path_x)
    dist(i) = sqrt( (path_x(i)- x_v)^2 + (path_y(i)-y_v)^2 );
end
[m,idx] = min(dist);