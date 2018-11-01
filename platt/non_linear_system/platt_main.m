function platt_main
    close all;
    params=setParams;
    bel(:,1)=params.b_init;
    if exist(['bel_des_' num2str(params.time.N) '_.mat'],'file')~=2
        [ctrl_des,bel_des,exitflag]=create_plan(bel,params);
        if exitflag==1 || exitflag==2
            save([ 'bel_des_' num2str(params.time.N) '_.mat'],'bel_des');
            save([ 'ctrl_des_' num2str(params.time.N) '_.mat'],'ctrl_des');
        end
    else
        load([ 'bel_des_' num2str(params.time.N) '_.mat'],'bel_des');
        load([ 'ctrl_des_' num2str(params.time.N) '_.mat'],'ctrl_des');
    end
    fig=draw_plan(bel_des,ctrl_des,params);
    %% trajectory following by LTV LQR
    for t=1:params.time.N-1
        ctrl(:,t) = lqr_control(bel(:,t),bel_des,ctrl_des,t,params);
        mean_pred=processDynamics(bel(1:params.num_robot_state,t),ctrl(:,t),params);
        y=repmat((-1+2*rand(1))*obsNoiseCov(mean_pred),params.num_ctrl,1);
        bel(:,t+1)= ekf(bel(:,t),ctrl(:,t),y,params);
    end
    ctrl(:,params.time.N)=0;
    figure(1);
    subplot(3,4,[1 2 3 5 6 7 9 10 11]);
    hold on;
    x = linspace(-2.5,10,100);
    z = obsNoiseCov(x);
    plot3(bel(1,:),bel(2,:),repmat(max(z),size(bel,2),1),'c','LineWidth',2,'DisplayName','B-LQR');
    drawnow;
    time=0:params.time.dt:params.time.T;
    subplot(3,4,8);
    hold on;
    plot(time,bel(3,:),'LineWidth',2,'DisplayName','B-LQR');
    drawnow;
    
    bel=[];
    ctrl=[];
    %% trajectory following by the robot
    bel(:,1)=[params.robot_init;5];
    bel_traj=bel;
%     ctrl_traj=zeros(params.num_ctrl,1);
    while norm(bel(1:2,end)-params.b_goal(1:2))>params.replan_thresh
        [ctrl_des,bel_des,exitflag]=create_plan(bel,params);
        if exitflag==1 || exitflag==2
            for t=1:params.time.N-1
                ctrl(:,t) = lqr_control(bel(:,t),bel_des,ctrl_des,t,params);
                mean_pred=processDynamics(bel(1:params.num_robot_state,t),ctrl(:,t),params);
                y=repmat((-1+2*rand(1))*obsNoiseCov(mean_pred),params.num_ctrl,1);
                bel(:,t+1)= ekf(bel(:,t),ctrl(:,t),y,params);
                if norm(bel(1:2,t+1)-bel_des(1:2,t+1))>params.replan_thresh
                    fprintf('replanning...\n');
                    if exist('ctrl_traj')==0
                        ctrl_traj=ctrl(:,1:end);
                    else    
                        ctrl_traj(:,end+1:end+t)=ctrl(:,1:end);
                    end
                    bel_traj(:,end+1:end+t)= bel(:,2:end);
                    bel(:,1)= bel(:,t+1);
                    bel(:,[2:end])=[];
                    ctrl=[];
                    break;
                end
            end
        else
            fprintf('no path found\n');
            return
        end
    end
    fprintf('found path...\n');
    bel_traj(:,end+1:end+params.time.N-1)=bel(:,2:end);
    if exist('ctrl_traj')==1
       ctrl_traj(:,end+1:end+params.time.N-1)=ctrl(:,1:end);
    else
        ctrl_traj=ctrl;
    end
    ctrl_traj(:,end+1)=0;
    figure(1);
    subplot(3,4,[1 2 3 5 6 7 9 10 11]);
    hold on;
    x = linspace(-2.5,10,100);
    z = obsNoiseCov(x);
    plot3(bel_traj(1,:),bel_traj(2,:),repmat(max(z),size(bel_traj,2),1),'g','LineWidth',2,'DisplayName','robot plan');
    drawnow;

    time=0:params.time.dt:(size(bel_traj,2)-1)*params.time.dt;
    subplot(3,4,8);
    hold on;
    plot(time,bel_traj(3,:),'LineWidth',2,'DisplayName','B-LQR-robot');
    drawnow;
    
end