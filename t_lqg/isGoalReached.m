function [goal_reached,goal_thresh]=isGoalReached(mean_curr,cov_curr,params)
% hardcoding
    bel_around(:,1)=params.b_goal(1)-0.3:0.01:params.b_goal(1)+0.3;
    bel_around(:,2)=params.b_goal(2)-0.3:0.01:params.b_goal(2)+0.3;
    [x,y]=meshgrid(bel_around(:,1),bel_around(:,2));
    bel_around=[];
    bel_around(:,1)=reshape(x,[],1);
    bel_around(:,2)=reshape(y,[],1);
    bel_around=bel_around(isInCircle(params.b_goal,0.3,bel_around),:);
    sigma=diag(repmat(cov_curr,1,2));
    f=mvncdf(bel_around,mean_curr',sigma);
    goal_reached=max(f)>=params.goal_thresh;
    goal_thresh=max(f);
end