% input vector
% x = [xi; xf; xi_dot; xf_dot; xi_2dot; xf_2dot]: constraints on the polynomial
% to, tf: start and end time

% output vector
% alpha = [a0 a1 a2 a3 a4 a5]: coefficients of the polynomial

function alpha = Get_Coefficient_Polynomials(x, to, tf)
% This function computes the coefficients of a degree 5 polynomial.
% call for test purposes
% x = [2;13;0;0;0;0]
% to = 0
% tf = 12
    
    n=size(x,1);
    T=zeros(n);
    T =     [1 to to^2 to^3 to^4 to^5;
            1 tf tf^2 tf^3 tf^4 tf^5;
            0 1 2*to 3*to^2 4*to^3 5*to^4;
            0 1 2*tf 3*tf^2 4*tf^3 5*tf^4;
            0 0 2 6*to 12*to^2 20*to^3;
            0 0 2 6*tf 12*tf^2 20*tf^3];
    alpha = T \ x;
    
end