function [Abarbar,Abarbar2,Bbarbar,Bbarbar2,Cbar_vel2,Cbar_pos2,Bbarbar3]=Matrices_acc_mtMPC(N,A,B,C,C_vel)
n     =   size(A,1);
m      =   size(B,2);
p      =   size(C,1);
ps     =   size(C(1:2,:),1);

for ind = 1:2*N-1
    Cbar_vel2((ind-1)*m+1:ind*m,(ind-1)*n+1:ind*n) = C_vel;
    Cbar_pos2((ind-1)*m+1:ind*m,(ind-1)*n+1:ind*n) = C;
end

Abar = zeros(n*N,n);
Bbar = zeros(n*N,m*N);
Bbar2 = zeros(n*N-n,m*N);

for ind = 1:N
    Abar((ind-1)*n+1:ind*n,1:n)=A^ind;
    for ind2 = 1:ind
        Bbar((ind-1)*n+1:ind*n,(ind2-1)*m+1:ind2*m)=A^(ind-ind2)*B;    
    end 
end

Abarbar = [Abar;Abar(n+1:end,:)];
Abarbar2 = [eye(n);Abar(1:n*N-n,:);Abar(1:n*N-n,:)];

Bbarbar = [Bbar zeros(N*n,m*N-m);Bbar(n+1:end,1:m) zeros(N*n-n,m*N-m) Bbar(n+1:end,m+1:end)];
Bbar2(1:n*N-n,1:m*N-m) = Bbar(1:N*n-n,1:m*N-m);
Bbarbar2 = [zeros(n,2*m*N-m);Bbar2 zeros(N*n-n,m*N-m);Bbar2(:,1:m) zeros(N*n-n,m*N-m) Bbar2(:,m+1:end)];
Bbarbar3 = [zeros(n,2*m*N-m);
            Bbar(1:end-n,:) zeros(N*n-n,m*N-m);
            Bbar(1:end-n,1:m) zeros(N*n-n,m*N-m) Bbar(1:end-n,m+1:end)];
        
end