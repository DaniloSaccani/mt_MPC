clear all
close all
clc


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

% Discretization
Ts = 1;
opt = c2dOptions('Method','zoh');
sysd1 = c2d(sys,Ts,opt);
A = sysd1.A;
B = sysd1.B;
C = sysd1.C;


x0 = [-5;0;0;0;0;0];

N = 5;

n     =   size(A,1);
m      =   size(B,2);
p      =   size(C,1);

Q = eye(p);

yref = [1;1;1];

lato =3;

vertex = [x0(1)+lato x0(2);
          x0(1)+2/3*lato x0(2)+2/3*lato;
          x0(1)-2/3*lato x0(2)+2/3*lato;
          x0(1)-2/3*lato x0(2)-2/3*lato;
          x0(1)+2/3*lato x0(2)-2/3*lato;
          x0(1)-lato x0(2);
          x0(1) x0(2)+lato;
          x0(1) x0(2)-lato];
      
[Ac,bc] = vert2con(vertex);

[Lambda_ys,Gamma_ys,Lambda_ye,Gamma_ye,Lambda_xi,Gamma_xi] = Traj_matrices_mtMPC(N,A,B,C);

nqxi    =   size(Ac,1);
nyc     =  size(Ac,2);
v_max = 1;
C_vel = [zeros(3,3) eye(3,3)];
for ind = 1:2*N
    Cbar_vel((ind-1)*p+1:ind*p,(ind-1)*n+1:ind*n) = C_vel;
end
for ind = 1:N+1
    
    Qbar((ind-1)*p+1:ind*p,(ind-1)*p+1:ind*p) = Q;
    Yref((ind-1)*p+1:ind*p,1)  =   yref;
    Acbar((ind-1)*nqxi+1:ind*nqxi,(ind-1)*nyc+1:ind*nyc)   =   Ac;
    bcbar((ind-1)*nqxi+1:ind*nqxi,1)                       =   bc;

end

f = x0'*Lambda_ye'*Qbar*Gamma_ye-Yref'*Qbar*Gamma_ye;
H       =   (Gamma_ye'*Qbar*Gamma_ye);
H       =   0.5*(H+H');


Aineq = [Acbar*Gamma_ys; 
        Cbar_vel*Gamma_xi;
        -Cbar_vel*Gamma_xi];
bineq = [bcbar-Acbar*Lambda_ys*x0;
        -Cbar_vel*Lambda_xi*x0+v_max*ones(size(Cbar_vel*Lambda_xi*x0));
        Cbar_vel*Lambda_xi*x0+v_max*ones(size(Cbar_vel*Lambda_xi*x0))];


%xSkpiuN = C_vel*(Lambda_xi(n*N+1:n*N+n,:)*x0)+C_vel*(Gamma_xi(n*N+1:n*N+n,:));
Aeq = C_vel*(Gamma_xi(n*N+1:n*N+n,:));
beq = -C_vel*(Lambda_xi(n*N+1:n*N+n,:)*x0);
%% QP options
options =   optimset('display','none');
U  =   quadprog(H,f,Aineq,bineq,Aeq,beq,[],[],[],options);
