% Decomposes a coodinate transformation X into [E 0; 0 E]*[1 0; -rX 1], and
% returns r - the position translation of the coordinate transformation
function r = X_to_r(X)
    E = X(1:3, 1:3) ;
    Erx = X(4:6, 1:3) ;
    rX = -inv(E)*Erx ;
    r = [rX(3,2);
         rX(1,3);
         rX(2,1)] ;
end