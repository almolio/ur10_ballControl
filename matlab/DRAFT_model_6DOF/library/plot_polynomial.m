function plot_polynomial(alpha, t0_w)
n = 100;

t = linspace(0,10,n);

x = zeros(n,3);

% alpha  = [0.6841  -0.2577    0.6115;
% 0.0102    0.0051    0.0037;
% -0.0112   -0.0056   -0.0040;
% 0.0048    0.0024    0.0017;
% -0.0006   -0.0003   -0.0002;
% 0.0000    0.0000    0.0000];

for i=1:n
    times = [1 t(i) t(i)^2 t(i)^3 t(i)^4 t(i)^5];
    for j=1:3
        x(i,j) = times * alpha(:,j); 
    end
end

x = x + t0_w';

plot3(x(:,1), x(:,2), x(:,3), 'm .','MarkerSize',10)
end