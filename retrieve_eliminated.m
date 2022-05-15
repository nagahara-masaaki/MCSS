function k=retrieve_eliminated(q,b,c)

% q + b'*k*c + c'*k'*b < 0

tt1=null([b;c]);
tt2=null([b;tt1']);
tt3=null([c;tt1']);
tt4=null([tt1,tt2,tt3]');
tn1=size(tt1,2);
tn2=size(tt2,2);
tn3=size(tt3,2);
tn4=size(tt4,2);
tt=[tt1,tt2,tt3,tt4];

ttq=tt'*q*tt;

ttq11=ttq(1:tn1,1:tn1);
ttq12=ttq(1:tn1,tn1+1:end);
ttq22=ttq(tn1+1:end,tn1+1:end);

tttq=ttq22-ttq12'/ttq11*ttq12;
tttq=tttq+eye(tn2+tn3+tn4);

tb34=[tt3,tt4]'*b';
tc24=[tt2,tt4]'*c';

k=-tb34\tttq((1:tn3+tn4)+tn2,[1:tn2,(1:tn4)+tn2+tn3])/tc24';

