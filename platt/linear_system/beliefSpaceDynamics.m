function bel_next=beliefSpaceDynamics(bel,ctrl,params)
 
    mean  = bel(1:params.num_robot_state);
    s     = bel(params.num_robot_state+1:end);
    sigma = diag(repmat(s,params.num_robot_state,1));
    mean_pred=processDynamics(mean,ctrl,params);
    bel_next(1:params.num_robot_state,1)=mean_pred;
    noiseCov=diag([obsNoiseCov(mean_pred),obsNoiseCov(mean_pred)]);
    lambda=params.domain.A*sigma*params.domain.A';
    sigma_next=lambda-lambda*params.domain.C'/(params.domain.C*lambda*params.domain.C'+noiseCov)*params.domain.C*lambda;
    
    assert(sigma_next(1,1)==sigma_next(2,2) && sigma_next(1,2)==sigma_next(2,1)&& sigma_next(1,2)==0,'a problem detected with sigma, try changing noiseCov');
    
    bel_next(params.num_robot_state+1,1)=sigma_next(1,1);
    
end