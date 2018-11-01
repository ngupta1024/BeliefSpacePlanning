function [belief_nominal, belief_posterior]= innerLayer(general_belief, u_nominal, gbs_params, robot_params,time_step)
    % belief- struct with mean and covariance of robot state at time tk
    % u_nominal- at time tk for tk to tk+L-1(Length =L)
    % belief_nominal.robot_states (length=k+L)
    belief_nominal.robot_states(:,1:time_step)=general_belief.robot_states(:,1:time_step);
    belief_nominal.world_states=general_belief.world_states;
    for iter=1:robot_params.lookahead
        belief_nominal.robot_states(:,end+1)=processDynamics(belief_nominal.robot_states(:,end),u_nominal(:,iter), robot_params);
    end
    % priors at t_k+1 to t_k+L (length=L)
    for iter=1:robot_params.lookahead
        [~,~,F(:,:,iter),~]=linearizeProcessDynamics(general_belief.robot_states(:,time_step+iter), u_nominal(:,iter),...
            belief_nominal.robot_states(:,time_step+iter), u_nominal(:,iter), robot_params);
        [~,~,H(:,:,iter)]=linearizeObsDynamics()
        belief_nominal.cov(:,:,l)=italics_A_'*italics_A_;
    end
    %% measurement stuff
    inliers=findPointsAroundRobot(hparams.camera_radius,map,mean_robot_states);
    for iter=1:size(inliers,1)    
        jacob_H(:,:,iter)=-inliers(iter,1)/d;
    end
end