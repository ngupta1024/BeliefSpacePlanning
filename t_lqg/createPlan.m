function [mean_des,ctrl_des,ef]=createPlan(mean_init,cov_init,params)
    ctrl  = zeros(params.num_ctrl,params.time.N); %1xN
%     bel_mean   = repmat(mean_init, 1, params.time.N);   %3xN
    bel_mean(1,:)=linspace(mean_init(1),params.b_goal(1),params.time.N);
    bel_mean(2,:)=linspace(mean_init(2),params.b_goal(2),params.time.N);
    initColl=[ctrl;bel_mean];
    
    global idx;
    
    idx.m = 3;
    idx.u = 1;
    
    objective = @objFunc;
    
    A   = [];
    b   = [];
    Aeq = [];
    beq = [];
    ub  = [repmat(params.domain.ctrl_max,2,params.time.N); inf(size(initColl,1)-2,params.time.N)];
    lb = -1*ub;
    
    non_lin_constraints=@nonlincst;
    
    options =optimoptions(@fmincon,'TolFun', 0.00000001,'MaxIter', 10000, ...
    'MaxFunEvals', 100000,'Display','iter', ...
     'DiffMinChange', 0.001,'Algorithm', 'sqp');
    [coll,fval,ef,op]=fmincon(objective, initColl, A,b,Aeq,beq, lb,ub,non_lin_constraints,options);
    ctrl_des=coll(1:idx.m-1,:);
    mean_des=coll(idx.m:end,:);
    
    function J=objFunc(init_coll)
        P(:,:,1)=diag(repmat(cov_init,1,2));
        J=0;
        for t=2:params.time.N
            P_=params.domain.A*P(:,:,t-1)*params.domain.A'+params.domain.G*params.process_noise_cov*params.domain.G';
            S=params.domain.C*P_*params.domain.C'+params.domain.M*obsNoiseCov(init_coll(idx.m:end,t))*params.domain.M';
            K=P_*params.domain.C'/(S);
            P(:,:,t)=(eye(2)-K*params.domain.C)*P_;
            J=J+trace(params.gains.Q*P(:,:,t)*params.gains.Q)+(init_coll(idx.u:idx.m-1,t-1)'*params.gains.R*init_coll(idx.u:idx.m-1,t-1));
        end
    end

    function [c,ceq]=nonlincst(initColl)
        bound_init=initColl(idx.m:end,1)-mean_init; %2x1
        ceq=[bound_init];
        for t=1:params.time.N-1
            ceq(end+1:end+params.num_robot_state)=initColl(idx.m:end,t+1)-processDynamics(initColl(idx.m:end,t),initColl(idx.u:idx.m-1,t),params,'NoNoise');
        end
        ceq(end+1:end+params.num_ctrl)=initColl(1:idx.m-1,end);
        c=[norm(initColl(idx.m:end,end)-params.b_goal)-params.goal_radius];
    end
end