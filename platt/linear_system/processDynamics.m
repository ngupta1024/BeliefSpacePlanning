function mean_next=processDynamics(mean,ctrl,params)
    mean_next=params.domain.A*mean+params.domain.B*ctrl*params.time.dt;
end