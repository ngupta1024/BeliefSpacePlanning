function [states_dot,states_next]=processDynamics(states, inputs, robot_params)
    % states- 3x1 
    % control- 2x1
    % robot_params- for dt
    x=states(1);
    y=states(2);
    theta=states(3);
    v=inputs(1);
    w=inputs(2);
    x_dot=v*cos(theta);
    y_dot=v*sin(theta);
    theta_dot=w;
    states_dot=[x_dot;y_dot;theta_dot];
    states_next=states+robot_params.dt*states_dot;        
end