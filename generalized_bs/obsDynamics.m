function measurement=obsDynamics(robot_state,params)
    % measurement- 2xm
    features=visibleFeatures(robot_state,params);%2xm
    for iter=1:size(features,2)
        measurement(:,iter)=[norm([features(1,iter)-robot_state(1);features(2,iter)-robot_state(2)]);...
            atan2(features(2,iter)-robot_state(2),features(1,iter)-robot_state(1))-robot_state(3)];
    end
end