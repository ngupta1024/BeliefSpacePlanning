function [measurement,H]=linearizeObsDynamics(states, lin_states,params)
    x=states(1);
    y=states(2);
    theta=states(3);
    features=visibleFeatures(robot_state,params);%2xm
    for i_iter=1:params.lookahead
        for j_iter=1:size(features,2)
            range=sqrt((features(1,j_iter)-lin_states(1,i_iter))^2+(features(2,j_iter)-lin_states(2,i_iter))^2);
            H(1,1,iter)=-(features(1,j_iter)-lin_states(1,i_iter))/range;
            H(1,2,iter)=-(features(2,j_iter)-lin_states(2,i_iter))/range;
            H(1,3,iter)=0;
            H(2,1,iter)=(-(features(2,j_iter)-lin_states(2,i_iter))^2/(features(1,j_iter)-lin_states(1,i_iter))^2)*(1/range^2);
            H(2,2,iter)=-1/range^2;
            H(2,3,iter)=-1;
        end
    end
end