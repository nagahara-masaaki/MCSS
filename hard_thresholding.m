function hv = hard_thresholding(lambda,v)

[m,n]=size(v);
mn = m*n;
hv = zeros(m,n);
for i = 1:mn
    if abs(v(i))<=lambda
        hv(i) = 0;
    else
        hv(i) = v(i);
    end
end
