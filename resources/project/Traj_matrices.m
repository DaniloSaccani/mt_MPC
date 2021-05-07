function [Lambda_y,Gamma_y,Lambda_xi,Gamma_xi]=Traj_matrices(M,A,B,C,D)
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
%           M           =   prediction horizon
%           A,B,C,D     =   discrete-time system matrices
%
%   OUTPUTS:
%           Lambda_y    =   Matrix relating Y to xi(t)
%           Gamma_y     =   Matrix relating Y to U
%           Lambda_xi   =   Matrix relating XI to xi(t)
%           Gamma_xi    =   Matrix relating XI to U

nxi     =   size(A,1);
nu      =   size(B,2);
ny      =   size(C,1);

Lambda_y  =   zeros(ny*(M+1),nxi);
Gamma_y   =   zeros(ny*(M+1),nu*M);
Lambda_xi =   zeros(nxi*(M+1),nxi);
Gamma_xi  =   zeros(nxi*(M+1),nu*M);

for ind = 1:M+1
   Lambda_y((ind-1)*ny+1:ind*ny,:)          =   C*A^(ind-1);
   Lambda_xi((ind-1)*nxi+1:ind*nxi,:)       =   A^(ind-1);
   for ind2 =   1:ind-1
       Gamma_xi((ind-1)*nxi+1:ind*nxi,(ind2-1)*nu+1:ind2*nu)=A^(ind-ind2-1)*B;
       if ind2==ind-1
           Gamma_y((ind-1)*ny+1:ind*ny,(ind2-1)*nu+1:ind2*nu)=C*B+D;
       else
           Gamma_y((ind-1)*ny+1:ind*ny,(ind2-1)*nu+1:ind2*nu)=C*A^(ind-ind2-1)*B;
       end
   end
end