% Function to compute dJdq product with the velocity Jacobian
% Given spatial vel = spatial Jacobian J times dq (bodyJac.m)
% Then, translational vel = translatitional Jacobian J times dq (bodyJac_vel.m)
%  v = Jv dq = (J2 - hat(P) J1) dq
% dv = dJv dq + Jv d2q
%    = dJ2 dq - hat(v) J1 dq - hat(P) dJ1 dq + Jv d2q
%    = (dJ2 - hat(v) J1 - hat(P) dJ1) dq + Jv d2q
%    = dJv dq + Jv d2q
% Output of this function is (dJv dq)
function dJvdq = dJdq_vel(model, q, dq, b, X)
    
end