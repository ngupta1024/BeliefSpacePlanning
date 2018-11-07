function params=setParams

    params.time.T=5;
    params.time.dt=0.1;
    params.time.N=params.time.T/params.time.dt +1;
    
    params.b_init.num_gauss=2;
    params.b_init.mean=[1.75 0; 2 0.5];
    params.b_init.cov=repmat(0.0625,params.b_init.num_gauss,1);
    params.b_goal=[0.5 1];
    
    params.num_robot_state=2;
    params.num_ctrl=2;
    params.num_sup_points=1000;
    
    params.domain.A=eye(params.num_robot_state);
    params.domain.B=eye(params.num_ctrl);
    params.domain.process_noise_var=0.01;
    
    params.cost.alpha=0.4;
    params.min_goal_prob=0.9;
    params.goal_radius=0.3;
    params.sample_prob_thresh=0.3;
    
end