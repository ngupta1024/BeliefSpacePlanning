function cov=obsNoiseCov(mean)
    cov=0.1.*(5-mean(1,:)).^2;
end
