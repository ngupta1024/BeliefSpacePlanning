function [mean_update,cov_update]=ekf(mean_curr,cov_curr,ctrl,y,params)
    sigma=diag([cov_curr,cov_curr]);
    mean_pred=processDynamics(mean_curr,ctrl,params,'noNoise');
    sigma_pred=params.domain.A*sigma*params.domain.A'+params.process_noise_cov;
    noiseCov=diag([obsNoiseCov(mean_pred),obsNoiseCov(mean_pred)]);
    S=params.domain.C*sigma_pred*params.domain.C'+noiseCov;
    K=sigma_pred*params.domain.C'/S;
    mean_update=mean_pred+K*y;
    sigma_update=(eye(size(sigma_pred))-K*params.domain.C)*sigma_pred;
    cov_update=sigma_update(1,1);
end