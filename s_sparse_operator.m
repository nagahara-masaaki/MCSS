

function y = s_sparse_operator(x,s)
%S_SPARSE_OPERATOR s-sparse operator
% Y = S_SPARSE_OPERATOR(X,S) sets all but the s largest (in magnitude)
% elements of vector X to zero
    [n,m]=size(x);
    y=zeros(n,m);
    [xs,indx]=sort(abs(x),1,'descend');
    indx_s = indx(1:s);
    y(indx_s)=x(indx_s);
end