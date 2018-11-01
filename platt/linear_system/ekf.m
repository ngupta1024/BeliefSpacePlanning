function bel_next=ekf(bel_curr,ctrl,y,params)
    mean=bel_curr(1:params.num_robot_state);
    sigma=diag([bel_curr(end),bel_curr(end)]);
    mean_pred=processDynamics(mean,ctrl,params);
    sigma_pred=params.domain.A*sigma*params.domain.A';
    noiseCov=diag([obsNoiseCov(mean_pred),obsNoiseCov(mean_pred)]);
    S=params.domain.C*sigma_pred*params.domain.C'+noiseCov;
    K=sigma_pred*params.domain.C'/S;
    mean_update=mean_pred+K*y;
    cov_update=(eye(size(sigma_pred))-K*params.domain.C)*sigma_pred;
    bel_next=[mean_update;cov_update(1,1)];
end