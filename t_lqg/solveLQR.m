function L=solveLQR(params)
    P=zeros(2,2,params.time.N);
    L=zeros(2,2,params.time.N-1);
    Q_squared=params.gains.Q'*params.gains.Q;
    P(:,:,params.time.N)=Q_squared;
    for t=params.time.N-1:-1:1
        P(:,:,t)=params.domain.A'*P(:,:,t+1)*params.domain.A...
            -params.domain.A'*P(:,:,t+1)*params.domain.B/(params.gains.R+params.domain.B'*P(:,:,t+1)*params.domain.B)*params.domain.B'*P(:,:,t+1)*params.domain.A...
            +Q_squared;
        L(:,:,t)=(params.gains.R+params.domain.B'*P(:,:,t)*params.domain.B)\params.domain.B'*P(:,:,t)*params.domain.A;
    end
end