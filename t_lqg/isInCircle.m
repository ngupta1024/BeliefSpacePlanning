function in_circle=isInCircle(center,radius,point)
    % center is 1x2
    % radius is scalar
    % point is nx2
    in_circle= ((point(:,1)-center(1)).^2+(point(:,2)-center(2)).^2-radius^2<=1e-6);
end