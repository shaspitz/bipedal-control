% Function to recover mass and COM offset (and Inertia)
% for body b
function M = total_mass(model)
    M = 0 ;
    for j=1:model.NB
        I = model.I{j} ;
        mass = I(6,6) ;
        M = M + mass ;
    end
end