%% MATLAB program for reduced-order controller design (Section 5)
% M. Nagahara and Y. Yamamoto,
% A survey on compressed sensing approach to systems and control
% Section 4
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
%[A,B1,B2,C1,C2,D11,D12,D21]=COMPleib('NN12');	
B=B2; C=C2;
D=zeros(size(C2,1),size(B2,2));

[n,m] = size(B);
[p,n] = size(C);

%%%%%%%
Ps = ss(A,B,C,D);
rb = rank(B);
rc = rank(C);

%% Order of controller
nc = 0; % static controller

%% Reduced order controller by minimizing the nuclear norm
X = sdpvar(n,n);
Y = sdpvar(n,n);

Bp = null(B')'; % B "perp"
Cp = null(C)'; % C' "perp"

In = eye(n);

epsil1 = 1e-8;
epsil2 = 1e-8;

LMI1 = [X,In;In,Y] - epsil1*zeros(2*n,2*n);
LMI2 = -Bp*[In,A]*[zeros(n,n),X;X,epsil2*X]*[In;A']*Bp'  - epsil1*eye(n-rb);
LMI3 = Cp*[A', -In]*[-epsil2*Y, Y;Y, zeros(n,n)]*[A;-In]*Cp' - epsil1*eye(n-rc);

LMI = [LMI1>=0,LMI2>=0,LMI3>=0];

%% Nuclear norm minimization
diagnostic = optimize(LMI,norm([X,In;In,Y],'nuclear'),ops);

X_L1 = value(X);
Y_L1 = value(Y);

%% Initialization for projection
% initial guess for (X,Y)
X00 = zeros(n,n);
Y00 = zeros(n,n);
% maximum number of iterations
max_iter = 1e2;
% stopping criterion
EPS = 1e-6;

%% Alternating Projetion by M.Nagahara et al. IEEE L-CSS 2022
X0 = X00; Y0 = Y00;
Px = X0; Py = Y0;
Qx = X0; Qy = Y0;

for k = 1:max_iter
    fprintf('(alt.proj) k=%d\n',k)

    % projection for rank condition
    [X1,Y1] = Proj_rank(X0,Y0,n+nc);    
    
    X1_P=X1; Y1_P=Y1;

    % feasibility check
    assign(X,X1); assign(Y,Y1);
    g1 = min(eig(double(LMI1)))+epsil1;
    g2 = min(eig(double(LMI2)))+epsil1;
    g3 = min(eig(double(LMI3)))+epsil1;

    if g1>0 & g2>0 & g3>0
        fprintf('normal exit from projection rank\n');
        break;
    end
    
    % projection for LMIs
    [X0,Y0] = Proj_LMI(X1,Y1,X,Y,LMI);

    X1_P=X0; Y1_P=Y0;

    % feasibility check
    assign(X,X0); assign(Y,Y0);
    g1 = min(eig(double(LMI1)))+epsil1;
    g2 = min(eig(double(LMI2)))+epsil1;
    g3 = min(eig(double(LMI3)))+epsil1;
    
    res2tmp=norm(X0*Y0-eye(n),'fro');
    % rank check
    if res2tmp<EPS
      fprintf('normal exit from iteration (after projection LMI)\n');
      break;
    end

end

%% Controller
% Nuclear norm
[K1,Pcl1] = controller_realization(Ps,X_L1,Y_L1,nc);
disp('Residual (nuclear norm)')
norm(X_L1*Y_L1-eye(n),'fro')
disp('Controller (nuclear norm)')
K1
disp('Poles (nuclear norm)')
disp(pole(Pcl1))

% Alternating projection
[K3,Pcl3] = controller_realization(Ps,X1_P,Y1_P,nc);
disp('Residual (alt.proj)')
norm(X1_P*Y1_P-eye(n),'fro')
disp('Controller (alt.proj)')
K3
disp('Poles (alt.proj)')
disp(pole(Pcl3))

return;