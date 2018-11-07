function var=calculateCov(mean_curr,cov_curr,mean_des,params)
    P(:,:,1)=diag(repmat(cov_curr,1,2));
    var(1)=cov_curr;
    for t=2:params.time.N
        P_=params.domain.A*P(:,:,t-1)*params.domain.A'+params.domain.G*params.process_noise_cov*params.domain.G';
        S=params.domain.C*P_*params.domain.C'+params.domain.M*obsNoiseCov(mean_des(:,t))*params.domain.M';
        K=P_*params.domain.C'/(S);
        P(:,:,t)=(eye(2)-K*params.domain.C)*P_;
        assert(P(1,1,t)==P(2,2,t) && P(1,2,t)==P(2,1,t)&& P(1,2,t)==0,'a problem detected with sigma, try changing noiseCov');
        var(t)=P(1,1,t);
    end
end