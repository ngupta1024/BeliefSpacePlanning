function env2d
close all;

%% robot and env robot_params
% a grid of 10m by 10m
% robot points in the direction of theta wrt to horizontal axis

robot_params.num_states=3;% x,y,theta
robot_params.num_inputs=2; % v, omega
robot_params.num_obs=2; %range, bearing
robot_params.dt=0.1; %100ms
robot_params.lookahead=10; %1s
robot_params.camera_radius=0.5*sqrt(2)+0.2;% something to play with
robot_params.map=genMapWithLandmarks('grid');
robot_params.view_angle=pi/3;%60 deg each side
robot_params.process_noise_inf=inv(eye(robot_params.num_states));
robot_params.obs_noise_inf=inv(eye(robot_params.num_obs));
%% paper robot_params
gbs_params.uncertainty_bound=5;
gbs_params.alpha_lb=0.6;
gbs_params.alpha(1)=0;
gbs_params.lambda=0.01;
gbs_params.process_noise_info=0.01*eye(4);
gbs_params.weights.M_u=10;
gbs_params.weights.M_x_=10;
gbs_params.weights.M_sigma_=10;
gbs_params.step_size=0.1;
%% initial conditions
% robot needs to go from [1.5,1.25,pi/2];
% to 3 goals: [5,1.25,0], [1.5,5,0], [8,8,0]
start_state=[2.5; 2.5;pi/4];
goal_state=[5;5.5;0];
start_cov=[1 0 0; 0 1 0; 0 0 1];
nominal_inputs=zeros(2,robot_params.lookahead);

%% plot
figure(1);
hold on;

% plot the map
plot(robot_params.map(:,1),robot_params.map(:,2),'LineStyle','none','Marker','*','Color',[0.7 0.7 0.7],'DisplayName','landmarks(hidden)');

% plot the start and goal states
plot(start_state(1),start_state(2),'bo','MarkerSize',10, 'MarkerFaceColor','b');
plot(goal_state(1),goal_state(2),'go','MarkerSize',10, 'MarkerFaceColor','b');

inliers=visibleFeatures(start_state,robot_params);
plot(inliers(1,:),inliers(2,:),'LineStyle','none','Marker','*','Color','k');

% plot the axis to denote rotation of robot (only plot x axis- where the robot is face pointing to)
% x_axis- (x,y)->(x+l*cos(),y+l*sin())
% y_axis- (x,y)->(x-l*sin(),y+l*cos())
% plot([start_state(1);start_state(1)+robot_params.camera_radius *cos(start_state(3))],[start_state(2);start_state(2)+robot_params.camera_radius *sin(start_state(3))],'r','LineWidth',2);
% plot([goal_state(1);goal_state(1)+robot_params.camera_radius *cos(goal_state(3))],[goal_state(2);goal_state(2)+robot_params.camera_radius *sin(goal_state(3))],'r','LineWidth',2);
%plot the camera angle
th = start_state(3)-robot_params.view_angle:0.01:start_state(3)+robot_params.view_angle;
xunit = robot_params.camera_radius * cos(th) + start_state(1);
yunit = robot_params.camera_radius * sin(th) + start_state(2);
plot(xunit, yunit,'g');

th = goal_state(3)-robot_params.view_angle:0.01:goal_state(3)+robot_params.view_angle;
xunit = robot_params.camera_radius * cos(th) + goal_state(1);
yunit = robot_params.camera_radius * sin(th) + goal_state(2);

plot(xunit, yunit,'g');

plot([start_state(1);start_state(1)+robot_params.camera_radius *cos(start_state(3)-robot_params.view_angle)],[start_state(2);start_state(2)+robot_params.camera_radius *sin(start_state(3)-robot_params.view_angle)],'g');
plot([start_state(1);start_state(1)+robot_params.camera_radius *cos(start_state(3)+robot_params.view_angle)],[start_state(2);start_state(2)+robot_params.camera_radius *sin(start_state(3)+robot_params.view_angle)],'g');
plot([goal_state(1);goal_state(1)+robot_params.camera_radius *cos(goal_state(3)-robot_params.view_angle)],[goal_state(2);goal_state(2)+robot_params.camera_radius *sin(goal_state(3)-robot_params.view_angle)],'g');
plot([goal_state(1);goal_state(1)+robot_params.camera_radius *cos(goal_state(3)+robot_params.view_angle)],[goal_state(2);goal_state(2)+robot_params.camera_radius *sin(goal_state(3)+robot_params.view_angle)],'g');

[x,y]=plot_covar(start_state,start_cov);
plot(x,y,'r');

% plot([x_robot; x_robot-sin(theta)],[y_robot; y_robot+cos(theta)],'g','LineWidth',2);
% plot([x_goal; x_goal-sin(theta_goal)],[y_goal; y_goal+cos(theta_goal)],'g','LineWidth',2);

legend('landmarks(hidden)','robot pos','goal','landmarks(visible)');
axis equal

%% algorithm
% state space is going to increase continously as more and more landmarks
% are seen
k=1;
m=size(inliers,1);
n=3*k+2*m;
general_belief.robot_state(:,1)=start_state;%num_statesxk
noisy_measurement=obsDynamics(robot_state,params)+ [mvnrnd([0,0],inv(robot_params.obs_noise_inf))]'; %2xm
for iter=1:m
    pose_landmark=estimateLandmarkState(general_belief.robot_state(:,end),noisy_measurement(:,iter));
    general_belief.world_poses(:,iter)=[pose_landmark.x;pose_landmark.y];%2xm
end
general_belief.inf_robot(:,:,1)=inv(start_cov); %3x3xk
general_belief.inf_world=repmat(robot_params.obs_noise_inf,1,1,m);%2x2xm
while(general_belief.robot_state(:,end)~=goal_state)
    time_step=2;
    u_next=genBelSpace(general_belief,nominal_inputs,gbs_params, robot_params,time_step);
    time_step=time_step+1;
end
%% functions
    function inlier=findPointsAroundRobot(radius,data,robot_pos)
        temp=(data(:,1)-robot_pos(1)).^2+(data(:,2)-robot_pos(2)).^2-radius^2;
        inliers_idx=find(temp<=0);
        inlier=data(inliers_idx,:);
    end
end