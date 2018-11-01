function [states_dot,states_next,A,B]=linearizeProcessDynamics(states, inputs, lin_states, lin_inputs, robot_params)
    % states- 3x1
    % inputs- 3x1
    % robot_params- for dt
    x_o=lin_states(1);
    y_o=lin_states(2);
    theta_o=lin_states(3);
    v_o=lin_inputs(1);
    w_o=lin_inputs(2);
    A=[1 0 -v_o*sin(theta_o);
       0 1 v_o*cos(theta_o);
       0 0 0];
    B=[cos(theta_o) 0;
        sin(theta_o) 0;
        0 1];
    states_dot=A*states+B*inputs;
    states_next=states+robot_params.dt*states_dot;
end