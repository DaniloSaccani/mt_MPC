function [Lambda_ys,Gamma_ys,Lambda_ye,Gamma_ye,Lambda_xi,Gamma_xi]=Traj_matrices_mtMPC(N,A,B,C)
% TRAJ_MATRICES Computes the matrices that relate the state and output
% trajectories of a LTI system to the initial condition and input sequence
% from the current time t up to t+N:
%                   XI  =   Lambda_xi*xi0+Gammma_xi*U
%                   Y   =   Lambda_y*xi0+Gammma_y*U
% where XI = [xi(t)^T,...,xi(t+M)^T]^T, Y = [y(t)^T,...,y(t+M)^T]^T and
% U = [u(t)^T,...,u(t+M)^T]^T
%
% The system is assumed to be modeled in discrete time:
%              xi(t+1)  =   A*xi(t)+B*u(t)
%                 y(t)  =   C*xi(t)+D*u(t)
%
%   INPUTS:
%           N           =   prediction horizon
%           A,B,C     =   discrete-time system matrices
%
%   OUTPUTS:
%           Lambda_y    =   Matrix relating Y to xi(t)
%           Gamma_y     =   Matrix relating Y to U
%           Lambda_xi   =   Matrix relating XI to xi(t)
%           Gamma_xi    =   Matrix relating XI to U


n     =   size(A,1);
m      =   size(B,2);
p      =   size(C,1);
ps     =   size(C(1:2,:),1);

Lambda_xi =   zeros(2*n*N,n);
Gamma_xi  =   zeros(2*n*N,2*m*N-m);
Lambda_y = 0;
Gamma_y = 0;

Abar = zeros(n*N,n);
Bbar = zeros(n*N,m*N);
C2 = eye(n);

for ind = 1:N
    Abar((ind-1)*n+1:ind*n,1:n)=A^ind;
    for ind2 = 1:ind
        Bbar((ind-1)*n+1:ind*n,(ind2-1)*m+1:ind2*m)=A^(ind-ind2)*B;    
    end 
end

% M = 2;                                                  % Number Of Times To Repeat
% Cr = repmat(C, 1, M);                                   % Repeat Matrix
% Cb = mat2cell(Cr, size(C,1), repmat(size(C,2),1,M));    % Create Cell Array Of Orignal Repeated Matrix
% Cbar2 = blkdiag(Cb{:});                                 % Original Matrix (Created)
% 
% M = N-1;                                                % Number Of Times To Repeat
% Cr = repmat(C, 1, M);                                   % Repeat Matrix
% Cb = mat2cell(Cr, size(C,1), repmat(size(C,2),1,M));    % Create Cell Array Of Orignal Repeated Matrix
% Cbar = blkdiag(Cb{:});
% 
% Cbbar_e = zeros(N*p+p,2*N*n);
% Cbbar_e(1:2*p,1:1:2*n)=Cbar2;
% Cbbar_e(2*p+1:end,N*n+n+1:end)=Cbar;
% 
% M = N+1;                                                       % Number Of Times To Repeat
% Cr = repmat(C(1:2,:), 1, M);                                   % Repeat Matrix
% Cb = mat2cell(Cr, size(C(1:2,:),1), repmat(size(C(1:2,:),2),1,M));    % Create Cell Array Of Orignal Repeated Matrix
% Cbbar_s2 = blkdiag(Cb{:});

%Cbbar_s = [Cbbar_s2 zeros(N*ps+ps,N*n-n)];

% Abbar = [Abar;Abar(n+1:end,1:n)];
Lambda_xi = [eye(n,n);Abar;Abar(n+1:end,1:n)];
% B1 = [Bbar zeros(N*n,m*N-m)];
% B2 = [Bbar(n+1:end,1:m) zeros(N*n-n,m*N-m) Bbar(n+1:end,m+1:end)];
% Bbbar = [Bbar zeros(N*n,m*N-m);Bbar(n+1:end,1:m) zeros(N*n-n,m*N-m) Bbar(n+1:end,m+1:end)];

Cbar_pos = zeros(N*p-p,N*n-n);
Cbar_pos_2 = zeros(2*p,2*n);
Cbar_pos_xy = zeros(N*ps-ps,N*n-n);
Cxy = [eye(2,p) zeros(2,p)];
for ind = 1:N-1
    Cbar_pos((ind-1)*p+1:ind*p,(ind-1)*n+1:ind*n) = C;
    Cbarbar_ex_t((ind-1)*n+1:ind*n,(ind-1)*n+1:ind*n) = C2;
    if ind<3
        Cbar_pos_2((ind-1)*p+1:ind*p,(ind-1)*n+1:ind*n) = C;
    end
    
end

for ind = 1:N+1
    Cbar_pos_xy((ind-1)*ps+1:ind*ps,(ind-1)*n+1:ind*n) = Cxy;
end

Cbarbar_ex = [eye(2*n) zeros(2*n,2*N*n-2*n);
          zeros(N*n-n,N*n+n)  Cbarbar_ex_t];

Cbarbar_pos = [Cbar_pos_2 zeros(2*p,2*N*n-2*n);
              zeros(N*p-p,N*n+n)  Cbar_pos];

Cbarbar_pos_xy = [Cbar_pos_xy zeros(N*ps+ps,N*n-n)];

Gamma_xi = [zeros(n,2*m*N-m);Bbar zeros(N*n,m*N-m);Bbar(n+1:end,1:m) zeros(N*n-n,m*N-m) Bbar(n+1:end,m+1:end)];


Lambda_ye = Cbarbar_ex*Lambda_xi;
Gamma_ye = Cbarbar_ex*Gamma_xi;
% Lambda_ye = Cbarbar_pos*Lambda_xi;
% Gamma_ye = Cbarbar_pos*Gamma_xi;

Lambda_ys = Cbarbar_pos_xy*Lambda_xi;
Gamma_ys = Cbarbar_pos_xy*Gamma_xi;

end