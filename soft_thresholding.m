function sv = soft_thresholding(lambda,v)

[m,n]=size(v);
mn = m*n;
sv = zeros(m,n);
for i = 1:mn
    if abs(v(i))<=lambda
        sv(i) = 0;
    else
        sv(i) = v(i) - sign(v(i))*lambda;
    end
end
