function platt_main
% dealing with multivariate mixture of gaussians
% receding horizon control in Belief space planning
    params=setParams;
    curr_time=1;
    % bel_curr is a struct
    %        - num_gauss
    %        - mean
    %        - cov
    bel(curr_time)=params.b_init;
    bel(curr_time).sigma=reshape(repmat(bel(curr_time).cov,1,params.num_robot_state),1,[],bel(curr_time).num_gauss);
    gm_curr=gmdistribution(bel(curr_time).mean,bel(curr_time).sigma); % assuming weight of each distribution is same
    while (~isGoalReached(gm_curr,params))
        %% make gm object
        support_points=sampleMixGauss(gm_curr,params);
        [cost_vec,ctrl_vec]=convexPlan(bel(curr_time),support_points,params);
        for tau=1:params.time.N-1
            z_new=obsDynamics(bel(curr_time+tau-1),ctrl_vec(:,curr_time));
            bel(end+1)=particleFilter(bel(curr_time+tau-1),z_new);
            if cost_vec(tau)<params.cost_thresh
                break;
            end
        end
        curr_time=curr_time+tau;
        bel(curr_time).sigma=reshape(repmat(bel(curr_time).cov,1,params.num_robot_state),1,[],bel(curr_time).num_gauss);
        gm_curr=gmdistribution(bel(curr_time).mean,bel(curr_time).sigma);
    end
end