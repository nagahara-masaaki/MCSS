function [Xopt,Yopt,info] = Proj_LMI_4(X0,Y0,X,Y,LMI1)

ops=sdpsettings;
ops=sdpsettings(ops,'verbose',0);
%Fsolver='sdpt3';
Fsolver='sedumi';
%Fsolver='mosek';

[mx,nx] = size(X0);
[my,ny] = size(Y0);

U  = blkdiag(X,Y);
U0 = blkdiag(X0,Y0);
gamm = sdpvar(1,1);
S11=sdpvar(mx,mx);
S12=sdpvar(my,my);
S1 =blkdiag(S11,S12);
%S1= sdpvar(mx+my,mx+my);
%S2= sdpvar(nx+ny,nx+ny);

M3 = [S1,U-U0; (U-U0)', eye(nx+ny)];
%M3 = [S1,U-U0; (U-U0)', S2];

LMI = [LMI1,M3>=0];

info=optimize(LMI,trace(S1),ops);
%info=optimize(LMI,trace(S1)+trace(S2),ops);

Xopt = value(X);
Yopt = value(Y);

end

