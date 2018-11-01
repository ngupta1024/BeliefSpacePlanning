function fig1=draw_plan(bel,ctrl,params)
    fig1=figure(1);
    set(gcf, 'Position', get(0, 'Screensize'));
    subplot(3,4,[1 2 3 5 6 7 9 10 11]);
    
    % plot the domain
    x = linspace(-2.5,10,100);
    y = x;
    z = obsNoiseCov(x);
    h=surf(x,y,repmat(z,size(x,2),1));
    oldcmap=colormap('gray');
    colormap( flipud(oldcmap) );
    shading interp
    view(2);
    title('Subplot 1: World')
    grid off;
    axis([-2.5 10 -2.5 10])
    hold on;
    
    % plot the robot current, bel_init and bel/robot_goal
    plot3(params.b_init(1),params.b_init(2),max(z),'ro','MarkerSize',10,'MarkerFaceColor','r');
    plot3(params.b_goal(1),params.b_goal(2),max(z),'bo','MarkerSize',10,'MarkerFaceColor','b');
    plot3(params.robot_init(1),params.robot_init(2),max(z),'go','MarkerSize',10,'MarkerFaceColor','g');
    plot3(bel(1,:),bel(2,:),repmat(max(z),size(bel,2),1),'k','LineWidth',2);
    drawnow;
    legend('','initial belief','goal','initial robot');
    
    subplot(3,4,4);
    time=0:params.time.dt:params.time.T;
    plot(time,bel(1,:),'LineWidth',2);
    hold on;
    plot(time,bel(2,:),'LineWidth',2);
    drawnow;
    legend('x','y');
    title('Subplot 2: x & y')
    
    subplot(3,4,8);
    plot(time,bel(3,:),'LineWidth',2);
    drawnow;
    legend('desired');
    title('Subplot 3: covariances')
     
    subplot(3,4,12);
    plot(time,ctrl(1,:),'LineWidth',2);
    hold on;
    plot(time,ctrl(2,:),'LineWidth',2);
    drawnow;
    legend('x velocity','y velocity');
    title('Subplot 4: control inputs')
    
end