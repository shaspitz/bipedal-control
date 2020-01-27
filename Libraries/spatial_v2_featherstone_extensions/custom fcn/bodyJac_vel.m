% Compute Jacobian for absolute velocity of body b wrt to base frame
% (combines rotation and translation velocites)
% The final argument serves as an additional transformation from body b.
% (For instance to compute the jacobian of a particular point on body b.)
function [Jv Jw_v] = bodyJac_vel(model, b, q, X)
    if(nargin < 4)
        X = eye(6) ;
    end
    P = X_to_r(bodypos(model, b, q, X)) ;
    J = bodyJac(model, b, q);%, X) ;
    Jv = J(4:6,:) - crm3(P)*J(1:3,:) ;
    
    Jw_v = [J(1:3,:) ; Jv] ;
end