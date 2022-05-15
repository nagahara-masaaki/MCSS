function [X1,Y1] = Proj_rank(X0,Y0,r)

% Projection of (X0,Y0) onto the set of
% {(X,Y): rank([X,I;I,Y]) <= r}
% via ADMM
% X and Y are n times n real matrices

%% Initialization
[n,n] = size(X0);
if r >= 2*n
    r = 2*n;
end
In = eye(n);

%% Projection onto the rank-r set via ADMM
Z = zeros(2*n,2*n);
U = Z;
rho = 100;
max_iter_rank = 1000;
EPS = 1e-15;
for kk = 1:max_iter_rank
    V = Z + U;
    V11 = V(1:n,1:n);
    V22 = V(n+1:2*n,n+1:2*n);
    X1 = 2/(2+rho)*(X0+rho/2*V11);
    Y1 = 2/(2+rho)*(Y0+rho/2*V22);
    T = [X1,In;In,Y1] - U;
    [UU,SS,VV] = svd(T);
    diag_tSS = s_sparse_operator(diag(SS),r);
    tSS = diag(diag_tSS);
    % Z = UU*tSS*VV';
    Z = UU*tSS*UU';
    residual = Z - [X1,In;In,Y1];
    U = U + residual;
    if norm(residual,inf) < EPS
        break;
    end
end

return