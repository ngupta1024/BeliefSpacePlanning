function [ctrl_des, bel_des, ef]=create_plan(bel_init,params)
    % direct transciption
    ctrl  = zeros(params.num_ctrl,params.time.N); %1xN
    bel   = repmat(bel_init, 1, params.time.N);   %3xN
    
    initColl=[ctrl;bel];
    
    global idx;
    
    idx.m = 3;
    idx.s = 5;
    idx.u = 1;
    
    objective = @objFunc;
    
    A   = [];
    b   = [];
    Aeq = [];
    beq = [];
    ub  = [repmat(params.domain.ctrl_max,2,params.time.N); repmat(inf,size(initColl,1)-2,params.time.N)];
    lb  = -1*ub;
    
    non_lin_constraints=@nonlincst;
    
    options =optimoptions(@fmincon,'TolFun', 0.00000001,'MaxIter', 10000, ...
    'MaxFunEvals', 100000,'Display','iter', ...
     'DiffMinChange', 0.001,'Algorithm', 'sqp');
    [coll,fval,ef,op]=fmincon(objective, initColl, A,b,Aeq,beq, lb,ub,non_lin_constraints,options);
    ctrl_des=coll(idx.u:idx.m-1,:);
    bel_des=coll(idx.m:end,:);
    
    function J=objFunc(initColl)
        J=0;
%         J=initColl(end,end)*params.gains.Lambda*initColl(end,end);
        for t=2:params.time.N
            J=J+(initColl([idx.m:idx.s-1],t)-params.b_goal(1:end-1))'*params.gains.Q*(initColl([idx.m:idx.s-1],t)-params.b_goal(1:end-1));
            J=J+(initColl(idx.u:idx.m-1,t)'-initColl(idx.u:idx.m-1,t-1)')*params.gains.R*(initColl(idx.u:idx.m-1,t)-initColl(idx.u:idx.m-1,t-1));
            J=J+initColl(end,t)*params.gains.Lambda*initColl(end,t);
        end
    end

    function [c,ceq]=nonlincst(initColl)
        bound_init=initColl(idx.m:end,1)-bel_init; %3x1
        bound_goal=initColl(idx.m:idx.s-1,end)-params.b_goal(1:params.num_robot_state); %2x1
        ceq=[bound_init;bound_goal];
        for t=1:params.time.N-1
            ceq(end+1:end+params.num_bel_state)=initColl(idx.m:end,t+1)-beliefSpaceDynamics(initColl(idx.m:end,t),initColl(idx.u:idx.m-1,t),params);
        end
        ceq(end+1:end+params.num_ctrl)=initColl(1:idx.m-1,end);
        c=[];
    end
    
end