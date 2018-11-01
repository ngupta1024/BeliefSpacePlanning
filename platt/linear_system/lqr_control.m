function ctrl_curr=lqr_control(bel_curr,bel_des,ctrl_des,timestep,params)
    S=zeros(params.num_bel_state,params.num_bel_state,params.time.N);
    S(:,:,params.time.N)=diag(repmat(params.gains.Lambda,params.num_bel_state,1));
    for t=params.time.N:-1:2
        ds_dm=finiteDiff('fs_mean',bel_des(:,t-1),ctrl_des(:,t-1),params);
        ds_ds=finiteDiff('fs_s',bel_des(:,t-1),ctrl_des(:,t-1),params);
        AA=[params.domain.A zeros(params.num_robot_state,1);
                repmat(ds_dm,1,2) ds_ds];
        BB=[params.domain.B;zeros(1,params.num_ctrl)];
        QQ=[params.gains.Q zeros(size(params.gains.Q,1),1);
            zeros(1,size(params.gains.Q,2)) zeros(1,1)];
        S(:,:,t-1)=QQ+AA'*S(:,:,t)*AA-AA'*S(:,:,t)*BB/(BB'*S(:,:,t)*BB+params.gains.R)*BB'*S(:,:,t)*AA;
        if(timestep==t-1)
            ctrl_curr=ctrl_des(:,t-1)-inv(BB'*S(:,:,t)*BB+params.gains.R)*BB'*S(:,:,t)*AA*(bel_curr-bel_des(:,t-1));
            ctrl_curr(ctrl_curr>params.domain.ctrl_max)=params.domain.ctrl_max;
            ctrl_curr(ctrl_curr<-1*params.domain.ctrl_max)=-1*params.domain.ctrl_max;
            break;
        end
    end 
end