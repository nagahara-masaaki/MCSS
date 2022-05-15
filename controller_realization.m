function [K,Pcl] = controller_realization(Ps,X1,Y1,nc);
% P: plant (ss)
% X1, Y1: LMI solutions
% nc: controller order

[A,B,C,D] = ssdata(Ps);

[fu,fs]=svd(Y1-inv(X1));
fv=fu*sqrt(fs(:,1:nc));
P=[eye(nc),fv';fv,Y1];

tq=P*blkdiag(zeros(nc),A)+(P*blkdiag(zeros(nc),A))';
tb=(P*blkdiag(eye(nc),B))';
tc=blkdiag(eye(nc),C);
k=retrieve_eliminated(tq,tb,tc);

ka=k(1:nc,1:nc);
kb=k(1:nc,nc+1:end);
kc=k(nc+1:end,1:nc);
kd=k(nc+1:end,nc+1:end);

K=ss(ka,kb,kc,kd);
kss=feedback(K,D);

Pcl=feedback(Ps,kss,1);
%pole(css)
