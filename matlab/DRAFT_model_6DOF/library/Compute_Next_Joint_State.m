% inputs
% q_curr: current state of the joints of size n x 1
% xf: final position in task space of size 3 x 1
% J: linear Jacobian evaluated for q_curr

% ouputs
% q_next: 3x1 joint angular position
% q_dot_next: 3x1 joint angular velocity

function [q_next, q_dot_next] = Compute_Next_Joint_State(DXef_0, q_curr, dt, J, gain)

    dx = gain * DXef_0 / dt;
    dq = J \ dx;
    q_next = q_curr + dq * dt;
    q_dot_next = dq;

end