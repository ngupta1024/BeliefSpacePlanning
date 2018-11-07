function params=setParams
    %% time
    params.time.T=5;
    params.time.dt=0.1;
    params.time.N=params.time.T/params.time.dt+1;
    
    %% gains
    params.gains.Q=chol(diag([5,5]));
    params.gains.R=diag([0.5,0.5]);
    
    %% states
    params.mean_init=[2;2];
    params.cov_init=5;
    params.b_goal=[0;0];
    params.goal_radius=0.1; %des
    params.goal_thresh=0.7;
    params.replan_thresh=0.2;
    params.num_robot_state=2;
    params.num_ctrl=2;
    params.process_noise_cov=0.1;
    %% light dark domain
    params.domain.A=eye(params.num_robot_state);
    params.domain.B=eye(params.num_ctrl);
    params.domain.C=eye(params.num_robot_state);
    params.domain.G=eye(params.num_robot_state);
    params.domain.M=eye(params.num_robot_state);
    params.domain.ctrl_max=2;
    
end