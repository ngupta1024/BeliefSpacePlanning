function params=setParams
%   ideally process noise and observation noise should be there
%   this paper by platt does not take into account any process noise
%   observational noise is dependent on the state in the light dark domain
%   params.process_noise = 0;
%   params.obs_noise     = obsNoise(state(1));

%% time

    params.time.dt             = 0.1;
    params.time.T              = 5;
    params.time.N              = params.time.T/params.time.dt+1;
    
%% gains

    params.gains.R             = diag([0.5,0.5]); % input deviation gain
    params.gains.Q             = diag([0.5,0.5]); % belief deviation gain
    params.gains.Lambda        = 2000; % gain for final covariance
    params.gains.Q_large       = params.gains.Lambda *eye(2);
%% states
    params.num_bel_state       = 3;
    params.num_ctrl            = 2;
    params.num_robot_state     = 2;
    params.b_init              = [2;2;5];
    params.b_goal              = [0;0;0.5];
    params.robot_init          = [2.5;0];
    
%% light dark domain
    params.domain.A=eye(params.num_robot_state);
    params.domain.B=eye(params.num_ctrl);
    params.domain.C=eye(params.num_robot_state);
    params.domain.ctrl_max=1;
%% others

    params.replan_thresh       = 0.5;
end