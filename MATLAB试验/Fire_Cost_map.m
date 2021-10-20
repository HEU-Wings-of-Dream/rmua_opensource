map = [1 : 100; 1 : 100];
A = 1;
x0 = 50;
y0 = 50;
sigmax = 50;
sigmay = 50;
for i = 1 : 1 : 100
    for j = 1 : 1 : 100
        map(i,j) = A * exp(-(((i-x0)*(i-x0))/(2*sigmax*sigmax)+((j-y0)*(j-y0))/(2*sigmay*sigmay)));
    end
end
surf(1:100, 1:100, map)