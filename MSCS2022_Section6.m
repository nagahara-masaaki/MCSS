%% MATLAB program for maximum hands-off control (Section 6)
% M. Nagahara and Y. Yamamoto,
% A survey on compressed sensing approach to systems and control
% Section 6

%% Initialization
clear;

%% System
s = tf('s');
Ps = 1/s^2/(s^2+1)*2;
[A,b,c,dd] = ssdata(Ps);
d = length(b); %system size
% initial state
x0 = ones(d,1);
% Horizon length
T = 10;

%% Time discretization
% Discretization size
n = 1000; % grid size
h = T/n; % discretization interval
% System discretization
[Ad,bd] = c2d(A,b,h);
% Matrix Phi
Phi = zeros(d,n);
v = bd;
Phi(:,end) = v;
for j = 1:n-1
    v = Ad*v;
    Phi(:,end-j) = v;
end
% Vector zeta
zeta = -Ad^n*x0;

%% Convex optimizaiton via ADMM
mu = 2*n+d;
Psi = [eye(n);eye(n);Phi];
%M = (Psi'*Psi)\Psi';
M = (0.5*eye(n) - 0.5*Phi'*inv(2*eye(d)+Phi*Phi')*Phi)*Psi';
sat = @(x) sign(x).*min(abs(x),1);

EPS = 1e-5; % if the residue < EPS then the iteration will stop
MAX_ITER = 100000; % maximum number of iterations
z = [zeros(2*n,1);zeta]; v = zeros(mu,1);
r = zeta;
k = 0;
gamma = 0.05;

tic
while (norm(r)>EPS) & (k < MAX_ITER)
    u = M*(z-v);
    z0 = soft_thresholding(gamma,u+v(1:n));
    z1 = sat(u+v(n+1:2*n));
    z2 = zeta;
    z = [z0;z1;z2];
    v = v + Psi*u - z;
    r = Phi*u - zeta;
    k = k + 1;
end
cpt=toc;

%% Plot
% Maximum hands-off control
figure;
plot(0:T/n:T-T/n,u);
title('Sparse control');

% States
[Phi,Ups] = get_mat2(Ad,bd,n);
Xe = zeros(n,d);
X2e = zeros(n,d);

X = Ups*x0 + Phi*u;

for j = 1:d
    Xe(:,j) = X(j:d:end)
end

figure;
te=0:h:h*(n-1);
Xed = downsample(Xe,30);
ted = downsample(te,30);
plot(te,Xe(:,1),'r-');
hold on
plot(te,Xe(:,2),'b-.');
plot(ted,Xed(:,3),'og-');
plot(ted,Xed(:,4),'sm-');
plot(te,u,'--k');
xlabel('time (sec)')
ylabel('x_i(t)')
title('state variables x_i(t)');
legend('x1','x2','x3','x4','u');


