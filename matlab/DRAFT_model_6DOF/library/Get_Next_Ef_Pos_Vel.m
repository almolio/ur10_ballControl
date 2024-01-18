% This function computes the position of the end effector at time 't'.

% input 
% alpha = [a0; a1; a2; a3; a4; a5]: coefficients of the polynomial
% ti: time

% output 
% p : 3x2 position and speed of end effector at time ti

function p = Get_Next_Ef_Pos_Vel(alpha, ti)

    t=[ 1 ti ti^2 ti^3 ti^4 ti^5;
        0 1 2*ti 3*ti^2 4*ti^3 5*ti^4];
    p = t * alpha;   
end