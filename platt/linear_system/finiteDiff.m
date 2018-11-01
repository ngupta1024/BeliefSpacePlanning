function deriv=finiteDiff(func_str,bel_curr,ctrl_curr,params)
    if strcmp('fs_mean',func_str)
        h=0.01;
        mean=bel_curr(1:params.num_robot_state);
        sigma=diag(repmat(bel_curr(end),params.num_robot_state,1));
        cov_diff=updateCov(mean+h,sigma,ctrl_curr)-updateCov(mean-h,sigma,ctrl_curr);
        assert(cov_diff(1,1)==cov_diff(2,2) && cov_diff(1,2)==cov_diff(2,1)&& cov_diff(1,2)==0,'a problem detected with sigma, try changing something');
        deriv=cov_diff(1,1)/2*h;
    
    elseif strcmp('fs_s',func_str)
        h=0.01;
        mean=bel_curr(1:params.num_robot_state);
        sigma=diag(repmat(bel_curr(end),params.num_robot_state,1));
        h_matrix=h*eye(size(sigma,1));
        cov_diff=updateCov(mean,sigma+h_matrix,ctrl_curr)-updateCov(mean,sigma+h_matrix,ctrl_curr);
        assert(cov_diff(1,1)==cov_diff(2,2) && cov_diff(1,2)==cov_diff(2,1)&& cov_diff(1,2)==0,'a problem detected with sigma, try changing something');
        deriv=cov_diff(1,1)/2*h;
    end
    
    function sigma_next=updateCov(mean,sigma,ctrl_curr)
        mean_pred=processDynamics(mean,ctrl_curr,params);
        lambda=params.domain.A*sigma*params.domain.A';
        noiseCov=diag([obsNoiseCov(mean_pred),obsNoiseCov(mean_pred)]);
        sigma_next=lambda-lambda*params.domain.C'/(params.domain.C*lambda*params.domain.C'+noiseCov)*params.domain.C*lambda;
    end
end