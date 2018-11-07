function mean_next=processDynamics(mean,ctrl,params,isNoise)
    if( strcmp(isNoise,'Noise'))
        mean_next=params.domain.A*mean+params.domain.B*ctrl*params.time.dt+(-1+2*rand(1))*params.process_noise_cov;
    else
        mean_next=params.domain.A*mean+params.domain.B*ctrl*params.time.dt;
    end
end