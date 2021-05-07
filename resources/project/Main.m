clear
clc
close all

%addpath('function') 


%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%% Model data %%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%

k1 = 0.7;
k2 = 1.3;

% Continuous time system
A_ct = [zeros(3,3) eye(3);
    -k1*eye(3) -k2*eye(3)];
B_ct = [zeros(3,3)
    k1*eye(3)];
C_ct = [eye(3,3) zeros(3,3)];
D_ct = 0;
sys = ss(A_ct,B_ct,C_ct,D_ct);
nx = size(A_ct,1); % Number of states
nu = size(B_ct,2); % Number of inputs
ny = size(C_ct,1);

% Discretization
Ts = 1;
opt = c2dOptions('Method','zoh');
sysd1 = c2d(sys,Ts,opt);
A = sysd1.A;
B = sysd1.B;
C = sysd1.C;
C_vel = [zeros(3,3) eye(3,3)];

% MPC data
Q = 2*eye(nx);
N = 20;

%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%% PARAMETERS %%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%
max_acc = 2; %m/s^2
v_max = 3; %m/s


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%% INITIAL STATES %%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
x0 = [-5;0;0;0;0;0];
x0_old = x0;
x_goal = [2;40;4;0;0;0];

[Lambda_ys,Gamma_ys,Lambda_ye,Gamma_ye,Lambda_xi,Gamma_xi] = Traj_matrices_mtMPC(N,A,B,C);
[Abarbar,Abarbar2,Bbarbar,Bbarbar2,Cbar_vel2,Cbar_pos2,Bbarbar3]=Matrices_acc_mtMPC(N,A,B,C,C_vel);

for ind = 1:2*N
    Cbar_vel((ind-1)*ny+1:ind*ny,(ind-1)*nx+1:ind*nx) = C_vel;
end
for ind = 1:N+1
    Qbar((ind-1)*nx+1:ind*nx,(ind-1)*nx+1:ind*nx) = Q;
    Yref((ind-1)*nx+1:ind*nx,1)  =   x_goal;
end

yalmip('clear')




% initialization collection of variables
XXfinal=[];

XXfinalQP = [];
XXfinalQP = [XXfinalQP x0];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%% PLOT INITIALIZATION %%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure(3);
pX1 = plot(x0(1),x0(2),'o');
hold on;
pX2 = plot(x0(1),x0(2),'o');
hold on;
pX3 = plot(x0(1),x0(2),'o');
hold on;
pX4 = plot(x0(1),x0(2),'o');
hold on;
pX0 = plot(x0(1),x0(2),'o');
ptarget = plot(x_goal(1),x_goal(2),'r*');

lato =16;

vertex = [x0(1)+lato x0(2);
          x0(1)+2/3*lato x0(2)+2/3*lato;
          x0(1)-2/3*lato x0(2)+2/3*lato;
          x0(1)-2/3*lato x0(2)-2/3*lato;
          x0(1)+2/3*lato x0(2)-2/3*lato;
          x0(1)-lato x0(2);
          x0(1) x0(2)+lato;
          x0(1) x0(2)-lato];

%subplot(211)
shp = alphaShape(vertex);
shp.Alpha = 10e2;
Polycon1 = plot(shp,'FaceColor','blue','FaceAlpha',0.1,'EdgeColor','none');

out1 = 0;

counter = 0;
normDiff = [];
normDiffS = [];
normDiffE = [];
while out1 == 0
     
%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%% OPTIMIZATION %%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%

vertex = [x0(1)+lato x0(2);
          x0(1)+2/3*lato x0(2)+2/3*lato;
          x0(1)-2/3*lato x0(2)+2/3*lato;
          x0(1)-2/3*lato x0(2)-2/3*lato;
          x0(1)+2/3*lato x0(2)-2/3*lato;
          x0(1)-lato x0(2);
          x0(1) x0(2)+lato;
          x0(1) x0(2)-lato];
      
[Ac,bc] = vert2con(vertex);

%%%%%% QP YALMIP
[x_next,u,diagnostics,objective,yalmipTime]=mt_MPC(x0,nu,N,max_acc,Q,A,B,Ac,bc,x_goal,k1,k2,v_max,Ts);
if diagnostics.problem == 0
 disp('Solver thinks it is feasible in ')
 disp(yalmipTime)
 %x0_old = x0;
 pippo = 1;
elseif diagnostics.problem == 1
 counter
 disp('Solver thinks it is infeasible')
 x_next = x0;
else
 disp('Something else happened')
end

%% QP QUADPROG
nqxi    =   size(Ac,1);
nyc     =  size(Ac,2);

for ind = 1:N+1
    Acbar((ind-1)*nqxi+1:ind*nqxi,(ind-1)*nyc+1:ind*nyc)   =   Ac;
    bcbar((ind-1)*nqxi+1:ind*nqxi,1)                       =   bc;
end


f = x0'*Lambda_ye'*Qbar*Gamma_ye-Yref'*Qbar*Gamma_ye;
H       =   (Gamma_ye'*Qbar*Gamma_ye);
H       =   0.5*(H+H');


Aacc = [(Cbar_vel2*Bbarbar./Ts-Cbar_vel2*Bbarbar2./Ts);
    -(Cbar_vel2*Bbarbar./Ts-Cbar_vel2*Bbarbar2./Ts)];
bacc = [max_acc+(-Cbar_vel2*Abarbar./Ts*x0+Cbar_vel2*Abarbar2./Ts*x0);
    max_acc-(-Cbar_vel2*Abarbar./Ts*x0+Cbar_vel2*Abarbar2./Ts*x0)];

%%% NOT working last method
% Aacc = [k1-k1.*Cbar_pos2*Bbarbar3-k2.*Cbar_vel2*Bbarbar3;
%         -k1+k1.*Cbar_pos2*Bbarbar3+k2.*Cbar_vel2*Bbarbar3];
% bacc = [max_acc+k1.*Cbar_pos2*Abarbar2*x0+k2.*Cbar_vel2*Abarbar2*x0;
%     max_acc-k1.*Cbar_pos2*Abarbar2*x0-k2.*Cbar_vel2*Abarbar2*x0];

Aineq = [Acbar*Gamma_ys;  %vincolo pos
        Cbar_vel*Gamma_xi; %vincolo vel
        -Cbar_vel*Gamma_xi; %vincolo vel
        Aacc]; 
bineq = [bcbar-Acbar*Lambda_ys*x0;
        -Cbar_vel*Lambda_xi*x0+v_max;
        Cbar_vel*Lambda_xi*x0+v_max; %vincolo vel
        bacc];

%xSkpiuN = C_vel*(Lambda_xi(n*N+1:n*N+n,:)*x0)+C_vel*(Gamma_xi(n*N+1:n*N+n,:));
Aeq = C_vel*(Gamma_xi(nx*N+1:nx*N+nx,:));
beq = -C_vel*(Lambda_xi(nx*N+1:nx*N+nx,:)*x0);
% QP options
options =   optimset('display','none','algorithm','interior-point');
tic
[U,fcostval,exitflag,output] =   quadprog(H,f,Aineq,bineq,Aeq,beq,[],[],[],options);
if exitflag == 1
    disp('Qp feasible in ')
    disp(toc)
    x_nextQP = A*x0+B*U(1:3);
elseif exitflag == -2
    disp('Qp not feasible')
    x_nextQP = x0;
else
    disp('Qp found something different')
end

QPtime = toc;
% solution debug
UQP = {};
count = 1;
for i =1:nu:length(U)
    UQP{count} = U(i:i+nu-1);
    count = count+1;
end
u_QP = [UQP{1}; UQP{2}; UQP{N+1}; UQP{3}; UQP{N+2};UQP{4};UQP{N+3};UQP{5};UQP{N+4}];
u_yalmip = [value(u{1}); value(u{2}); value(u{3}); value(u{4}); value(u{5});value(u{6}); value(u{7});value(u{8});value(u{9})];
diffE = [UQP{N+1}-value(u{3}); UQP{N+2}-value(u{5});UQP{N+3}-value(u{7});UQP{N+4}-value(u{9})];
diffS = [UQP{2}-value(u{2}); UQP{3}-value(u{4});UQP{4}-value(u{6});UQP{5}-value(u{8})];
diff = u_QP-u_yalmip;
normDiff=[normDiff;max(abs(diff))];
normDiffS=[normDiffS;max(abs(diffS))];
normDiffE=[normDiffE;max(abs(diffE))];



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%% TRAJECTORIES GENERATION %%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
x1 = A*x0+B*value(u{1});
x2 = A*x0+B*value(u{1});
x3 = A*x0+B*UQP{1};
x4 = A*x0+B*UQP{1};
XX1 = [];
XX2 = [];
XX3 = [];
XX4 = [];
XX1 = [XX1;x0';x1']; 
XX2 = [XX2;x0';x2'];
XX3 = [XX3;x0';x3']; 
XX4 = [XX4;x0';x4'];
for k = 2:2:2*N-2
    x1 = A*x1+B*value(u{k}); %SAFE
    XX1 = [XX1;x1'];
    x2 = A*x2+B*value(u{k+1}); %EXPLOIT
    XX2 = [XX2;x2'];
end
for k = 2:N
    x3 = A*x3+B*UQP{k};   %SAFE
    XX3 = [XX3;x3'];
    x4 = A*x4+B*UQP{N+k-1}; %explo
    XX4 = [XX4;x4'];
end
XX1 = value(XX1);
XX2 = value(XX2);

%%
% plot trajectories
figure(3)
%%%%%%%%%%%%%%%%%%%%%%%%
%%%% FIGURE UPDATE %%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%
delete(pX1);
delete(pX2);
delete(pX3);
delete(pX4);
delete(pX0);
delete(ptarget);
delete(Polycon1);

%%%%%%%%%%%%%%%%
%%%%% PLOT %%%%%
%%%%%%%%%%%%%%%%
shp = alphaShape(vertex);
shp.Alpha = 10e2;
Polycon1 = plot(shp,'FaceColor','blue','FaceAlpha',0.1,'EdgeColor','none');


pX1 = plot(XX1(:,1),XX1(:,2),'-*b'); hold on;
pX2 = plot(XX2(:,1),XX2(:,2),'-*g'); hold on;
% pX1 = plot(x0(1),x0(2),'o'); hold on;
% pX2 = plot(x0(1),x0(2),'o'); hold on;
pX3 = plot(XX3(:,1),XX3(:,2),'-*r'); hold on;
pX4 = plot(XX4(:,1),XX4(:,2),'-*y'); hold on;
legend('','safe yalmip','exp yalmip','safe QP','exp QP','','')
pX0 = plot(x0(1),x0(2),'o');
%ptarget = plot(x_goal(1),x_goal(2),'r*');
title({
    ['Multi-trajectory MPC J:' num2str(objective) 'state:' ] 
    ['x = ' num2str(x0(1)) ' y = ' num2str(x0(2)) ' vx = ' num2str(x0(4)) ' vy = ' num2str(x0(5)) ] 
    });
xlabel('x [m]')
ylabel('y [m]')
pause(0.5);

view(0,90);


if norm(x0-x_goal,1)<0.01
    out1 = 1;
end

%%%%%%%%%%%%%%%%%%%%%%
%%%%% TIME UPDATE %%%%
%%%%%%%%%%%%%%%%%%%%%%
%x0 = x_next;
x0 = x_nextQP;
counter = counter+1;
% storing trajectory
XXfinal = [XXfinal x0];
XXfinalQP = [XXfinalQP x3];
pause(2)
end
figure(3)
plot(XXfinal(1,:),XXfinal(2,:),'*b')
hold on;
plot(XXfinalQP(1,:),XXfinalQP(2,:),'*r')

figure
plot(normDiff);ylabel('max |difference|');xlabel('iteration')
figure
plot(normDiffE);ylabel('max |difference| exploit');xlabel('iteration')
figure
plot(normDiffS);ylabel('max |difference| safe');xlabel('iteration')