function kl_val=klDivergence(mean_curr,cov_curr,mean_des,cov_des,params)
    est_error=mean_curr-mean_des;
    sigma_curr=diag(repmat(cov_curr,1,2));
    sigma_des=diag(repmat(cov_des,1,2));
    kl_val=-0.5*params.num_robot_state;
    kl_val=kl_val+1/4*(trace(sigma_curr\sigma_des)+trace(sigma_des\sigma_curr));
    kl_val=kl_val+1/4*(est_error'/(sigma_curr)*est_error);
    kl_val=kl_val+1/4*(est_error'/(sigma_des)*est_error);
end