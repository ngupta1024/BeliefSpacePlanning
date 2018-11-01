function z=measurementDynamics(mean,params)
    % noise only in x direction
    z=params.domain.C*mean+(-1+2*rand(1))*obsNoiseCov(mean);
end