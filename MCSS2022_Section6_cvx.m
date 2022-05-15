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

%% Convex optimizaiton via CVX
cvx_begin
 variable u(n)
 minimize norm(u,1)
 subject to 
   Phi*u == zeta;
   norm(u,inf) <= 1;
cvx_end

%% Plot
figure;
plot(0:T/n:T-T/n,u);
title('Sparse control');


