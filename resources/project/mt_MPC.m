function [x_next,u,diagnostic,objective,yalmipTime] = mt_MPC(x0,nu,N,max_acc,Q,A,B,Ac,bc,x_goal,k1,k2,v_max,Ts)
%% Construction of the optimization problem
%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%% OPTIM. VARIABLE %%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%
u = sdpvar(repmat(nu,1,2*N-1),ones(1,2*N-1));
%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%% INITIALIZATION %%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%
constraints = [];
objective = 0;

%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%% FIRST TIME STEP %%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%
x1 = A*x0 + B*u{1};
x2 = A*x0 + B*u{1};

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%% OBJECTIVE FIRST TIME STEP %%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
objective = objective + norm(Q*(x2-x_goal),2).^2;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%% CONSTRAINTS FIRST TIME STEP %%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% BUILDING OF THE CONSTRAINTS
constraints = [constraints,...
%               -max_acc <= k1*(u{1}-x1(1:3))-k2*x1(4:end),...
%               k1*(u{1}-x1(1:3))-k2*x1(4:end) <= max_acc,...
               -max_acc <= (x1(4:end)-x0(4:end))./Ts,...
               (x1(4:end)-x0(4:end))./Ts<= max_acc,...
               Ac*x1(1:2)<=bc,...
               Ac*x0(1:2)<=bc,...
               x1(4:end)<=v_max.*ones(3,1),...
               x1(4:end)>=-v_max.*ones(3,1)];


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%% PREDICTION LOOP %%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
ind = 1;
for k = 2:2:2*N-1
    %%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%% SIMULATION %%%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%
    x11 = A*x1 + B*u{k};
    x22 = A*x2 + B*u{k+1};
    %%%%%%%%%%%%%%%%%%%%%%%%%
    %%%% COST FUNCTION %%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%
    objective = objective + norm(Q*(x22-x_goal),2).^2;    
 
    %%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%% CONSTRAINTS %%%%%%%
    %%%%%%%%%%%%%%%%%%%%%%%%%
    constraints = [constraints,...
%                       -max_acc <=  k1*(u{k}-x11(1:3))-k2*x11(4:end),...
%                        k1*(u{k}-x11(1:3))-k2*x11(4:end)<= max_acc,...
%                       -max_acc <=  k1*(u{k+1}-x22(1:3))-k2*x22(4:end),...
%                       k1*(u{k+1}-x22(1:3))-k2*x22(4:end)<= max_acc,...
                     -max_acc <=  (x11(4:end)-x1(4:end))./Ts,...
                     (x11(4:end)-x1(4:end))./Ts<= max_acc,...
                     -max_acc <= (x22(4:end)-x2(4:end))./Ts,...
                     (x22(4:end)-x2(4:end))./Ts <= max_acc,...
                     Ac*x11(1:2)<=bc,...
                     x11(4:end)<=v_max.*ones(3,1),...
                    x11(4:end)>=-v_max.*ones(3,1),...
                     x22(4:end)<=v_max.*ones(3,1),...
                    x22(4:end)>=-v_max.*ones(3,1)];

    %%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%% TIME UPDATE %%%%%% 
    %%%%%%%%%%%%%%%%%%%%%%%%%
    x1 = x11;
    x2 = x22;
    ind = ind+1;
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%% TERMINAL COST %%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%
%objective = objective + norm(Q(1:3,1:3)*(x22(1:3)-x_goal(1:3)),2);
constraints = [constraints,...
              x1(end-2:end)==[0;0;0]];

%% Optimization
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%% OPTIMIZATION %%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%ops = sdpsettings('verbose',2);
ops = sdpsettings('verbose',0,'cachesolvers','1','solver','quadprog');%'solver','mosek',
tic
diagnostic = optimize(constraints,objective,ops);
yalmipTime = toc;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%% OPT VALUE COMPUTATION %%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
inputOpt = value(u{1});
u = value(u);
x1 = A*x0+B*value(u{1});
x_next = x1;
objective = value(objective);
end



