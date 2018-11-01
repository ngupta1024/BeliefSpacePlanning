function features=visibleFeatures(robot_state,params)
    % robot can only see in the 120 degree angle window (+60 to -60)
    % robot_pos- 3x1
    % params - for map
    % features - 2xm (m landmarks visible) 
    x=robot_state(1);
    y=robot_state(2);
    theta=robot_state(3);
    temp=(params.map(:,1)-robot_state(1)).^2+(params.map(:,2)-robot_state(2)).^2-params.camera_radius^2;
    features=params.map(temp<=0,:); %mx2
    features=features((atan2(features(:,2)-y,features(:,1)-x)>theta-params.view_angle & atan2(features(:,2)-y,features(:,1)-x)<theta+params.view_angle),:);
    features=features';
end