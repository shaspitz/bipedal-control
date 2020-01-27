% Decomposes a coodinate transformation X into [E 0; 0 E]*[1 0; -rX 1], and
% returns E converted to rpy - the rotation of the coordinate transformation
function rpy = X_to_rpy(X)
    E = X(1:3, 1:3) ;
    [phi,theta,psi] = RotToRPY_ZXY(E) ;
    rpy = [phi,theta,psi] ;
end