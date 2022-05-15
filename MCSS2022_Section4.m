%% MATLAB program for sparse feedback gain design (Section 4)
% M. Nagahara and Y. Yamamoto,
% A survey on compressed sensing approach to systems and control
% Section 3
%
% To run this program, you need to instal
% YALMIP: https://yalmip.github.io/
% SeDuMi: https://sedumi.ie.lehigh.edu/
% Compleib: http://www.complib.de/

%% Initialization
clear;
ops=sdpsettings;

%% Syste matrices
% COMPleib benchmark
[A,B1,B2,C1,C2,D11,D12,D21]=COMPleib('AC2');	
B=B2; C=C2;
D=zeros(size(C2,1),size(B2,2));
[n,m] = size(B);
[p,n] = size(C);

%% Sparse feedback gain
% L1 relaxation by Polyak et al. ECC 2013
epsil = 1e-6; % epsilon for LMI

Y1 = sdpvar(m,n);
P1 = sdpvar(n,n);
M1 = [A*P1 + P1*A' + B*Y1 + Y1'*B' <= -epsil, P1 >= epsil];
optimize(M1,norm(Y1,1)) % minimizing L1 norm of Y1

% Alternating projection by Nagahara et al. IEEE L-CSS, 2022
% initialization
max_itr = 10; % maximum iteration
Z0 = value(Y1); % initial guess
V0 = zeros(m,n); % initial value
W0 = zeros(m,n); % initial value

sp = 1; % sparsity

for k = 1:max_itr
    % step 1: hard thresholding
    Y0 = s_sparse_operator_mat(Z0+V0,sp);
    V0 = Z0 + V0 - Y0;
    % step 2: projection onto the LMI set
    Z = sdpvar(m,n);
    P = sdpvar(n,n);
    LMI1 = [A*P + P*A' + B*Z + Z'*B' <= -epsil];
    LMI2 = [P >= epsil];
    S = sdpvar(n,n);
    M = [S, (Z-(Y0+W0))'; Z-(Y0+W0), eye(m)];
    LMI = [M>=0, LMI1, LMI2];
    optimize(LMI,trace(S));
    Z0 = value(Z);
    W0 = Y0 + W0 - Z0;
end
Y0 = Z0;
P0 = value(P); % Lyapunov variable

%% Sparse gain
disp('***Sparse feedback gain***')
disp('***Y by L1 norm heuristic')
disp(value(Y1)) % L1 optimal
disp('***Y by Alternating projection')
disp(Y0) % proposed

