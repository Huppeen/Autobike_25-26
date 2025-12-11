
function roll_ref =  MPC_estimation(e1,e2,v)
    %persistent A B C D N Q Pf R F G h_mpc lambda lf lr g Q_bar R_bar H rowA colA rowB colB Aeq1 Aeq2 Aeq 
    bike_params=LoadBikeParameters("green");
    load MPC_params_train.mat;

    x=[e1 e2]';
    tic
   % if isempty(A)
    % Unpack parameters
        g  = 9.81;
        lr = bike_params.lr;
        lf = bike_params.lf;
        lambda = bike_params.lambda;
        
        % MPC controller
        A=mpc_outer_params.Ad;
        B=mpc_outer_params.Bd;
        C=mpc_outer_params.Cd;
        D=mpc_outer_params.Dd;
        N=mpc_outer_params.N;
        Q=mpc_outer_params.Q;
        Pf=mpc_outer_params.Pf;
        R=mpc_outer_params.R;
        F=mpc_outer_params.F;
        G=mpc_outer_params.G;
        h_mpc=mpc_outer_params.h;
        
        
        %Assemble the Q_bar and R_bar
        Q_bar = blkdiag(kron(eye(N-1),Q),Pf);
        R_bar = kron(eye(N),R);
        %quadratic objective form
        H=blkdiag(Q_bar,R_bar);
        %f=[]; %f is empty due to the lack of linear terms in cost func
        
        %equality constraints
        [rowA,colA]=size(A);
        [rowB,colB]=size(B);
        
        Aeq1= kron(diag(ones(N-1,1),-1),-A)+ kron(eye(N), eye(rowA));
        
        Aeq2 = kron(eye(N),-B);
        Aeq  = [Aeq1 Aeq2];
    %end
    beq  = [A*x;zeros(colA*(N-1),1)];
    
    %inequallity
    Ain  = [F G];
    bin  = h_mpc;
    
    coder.extrinsic('quadprog');
    coder.extrinsic('optimset');
    % max time tunable...
    maxTime=10;
    options = optimset('Display', 'off','TolX',1e-2,'TolFun', 1e-3,'MaxTime',maxTime);
    
    EXITFLAG = 0;
    
    [Z,VN,EXITFLAG] = quadprog(2*H,[],Ain,bin,Aeq,beq,[],[],[],options);
    
    delta_ref = 0;
    delta_ref=Z(colA*N+1);
    
    if delta_ref>=pi/9
        delta_ref=pi/9;
     
    end
    if delta_ref<=-pi/9
        delta_ref=-pi/9;
    end
    eff_delta_ref = delta_ref*sin(lambda);
    roll_ref = -1*atan((tan(eff_delta_ref)*(v^2/(lr+lf)))/g);
end

N=100000;
v=2;
Training_data=zeros(N,3);
a = -20;
b = 20;
c= -180;
d= 180;
tic
for k = 1:N
    e1 = a + (b-a).*rand(1);
    e2 = deg2rad(c + (d-c).*rand(1));
    roll_ref = MPC_estimation(e1,e2,v);
    Training_data(k, :) = [e1, e2, roll_ref];
end
toc
