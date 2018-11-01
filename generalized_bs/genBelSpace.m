function [u_next]=genBelSpace(general_belief,u_nominal,gbs_params, robot_params,time_step)
%% outer layer
% @param: belief- struct with mean and covariance of all time steps till
%         current
% @param: u_nominal lookahead nominal control
% @param: gbs_params- alpha,beta,weights,grad step size 
%
% @return: optimal control over horizon lag
%  

[belief_nominal,belief_posterior]=innerLayer(general_belief,u_nominal, gbs_params, robot_params,time_step);

    
    
end