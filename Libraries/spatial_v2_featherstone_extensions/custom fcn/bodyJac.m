% Compute Jacobian of body b wrt to base frame
% The final argument serves as an additional transformation from body b.
% (For instance to compute the jacobian of a particular point on body b.)
function J = bodyJac(model, b, q)%, X)
    e = zeros(1, model.NB) ;
    e(model.kappa{b}) = 1 ;
    
    J = zeros(6, model.NB) ;
    if(isa(q, 'sym'))
        J = sym(J) ;
    end
    
    for i=1:model.NB
        if e(i)
            [XJ S] = jcalc(model.jtype{i}, q(i)) ;
            Xa{i} = XJ * model.Xtree{i} ;
            if(model.parent(i) ~= 0)
                Xa{i} = Xa{i} * Xa{model.parent(i)} ;
            end
%             if(i == b && nargin == 4)
%                 Xa{i} = X*Xa{i} ;
%             end
            J(:,i) = Xa{i} \ S ;
        end
    end
end