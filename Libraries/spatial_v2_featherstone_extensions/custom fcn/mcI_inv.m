% Function to recover mass and COM offset (and Inertia)
% for body b
function [mass com_offset I] = mcI_inv(model, b)
    if(nargin == 1)
        I = model ;
    else
        I = model.I{b} ;
    end
    
    mass = I(6,6) ;
    if(mass > 0)
        com_offset = [I(3,5) ;
                      I(1,6) ;
                      I(2,4)] / mass ;
    else
        com_offset = zeros(3,1) ;
    end
    
    
    mC = I(1:3,4:6);
    I = I(1:3,1:3) - mC*mC'/mass;
end