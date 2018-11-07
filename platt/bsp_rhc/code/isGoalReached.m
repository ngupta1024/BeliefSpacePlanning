function goal_reached = isGoalReached(gm,params)
    bel_around(:,1)=params.b_goal(1)-params.goal_radius:0.01:params.b_goal(1)+params.goal_radius;
    bel_around(:,2)=params.b_goal(2)-params.goal_radius:0.01:params.b_goal(2)+params.goal_radius;
    [x,y]=meshgrid(bel_around(:,1),bel_around(:,2));
    bel_around=[];
    bel_around(:,1)=reshape(x,[],1);
    bel_around(:,2)=reshape(y,[],1);
    bel_around=bel_around(isInCircle(params.b_goal,params.goal_radius,bel_around),:);
    f=cdf(gm,bel_around);
    goal_reached=f(end)>=params.min_goal_prob;
end

