% Compute position and orientation of body b
% The final argument serves as an additional transformation from body b.
% (For instance to compute the position of a particular point on body b.)
function X = bodypos(model, b, q, X)
    if(nargin < 4)
        X = eye(6) ;
    end
    while b > 0
        XJ = jcalc(model.jtype{b}, q(b)) ;
        X = X * XJ * model.Xtree{b} ;
        b = model.parent(b) ;
    end
end