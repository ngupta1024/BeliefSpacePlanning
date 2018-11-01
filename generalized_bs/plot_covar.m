function [x,y]=plot_covar(start_state,state_cov)    
    t = linspace(0,2*pi,1000);
    state_cov=state_cov(1:2,1:2);
    a=state_cov(1,1)/max(state_cov(1,1),state_cov(2,2));
    b=state_cov(2,2)/max(state_cov(1,1),state_cov(2,2));
    state_cov(1,1)=1;
    state_cov(2,2)=1;
    [V,D]=eig(state_cov);
    v1=V(:,diag(D)==max(diag(D)));
    theta0 = atan2(v1(2),v1(1))+start_state(3);
    x =  start_state(1) + a*cos(t)*cos(theta0) - b*sin(t)*sin(theta0);
    y = start_state(2) + b*sin(t)*cos(theta0) + a*cos(t)*sin(theta0);
%     x = start_state(1)+a*sin(t+theta0);
%     y = start_state(2)+b*cos(t);
end