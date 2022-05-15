function [Phi,Ups] = get_mat2(A,B,N);
%Upsilon
Ups=A;
for i=2:N
   Ups=[Ups;A^(i)];
end
%Phi
Phi=[];
for j=1:N
   aux=[];
   for i=1:j-1
      aux=[aux;0*B];
   end
   for i=j:N
      aux=[aux;A^(i-j)*B];
   end
   Phi=[Phi,aux];
end
return;
