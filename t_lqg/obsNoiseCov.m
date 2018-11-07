function cov=obsNoiseCov(mean)
    cov=0.1.*(3-mean(1,:)).^2;
end