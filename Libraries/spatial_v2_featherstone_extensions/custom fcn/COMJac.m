% Function to compute the jacobian of COM of the systema
function J = COMJac(model, q)
    % First implementation - highly inefficient - computes jacobians of
    % each individual link COM jacobian and then does a weighted mean
    
    mass_sum = 0 ;
    J = zeros(6, length(q)) ;
    for j=1:model.NB
        [mass] = mcI_inv(model, j) ;
        mass_sum = mass_sum + mass ;
        body_COM_jac = bodyJac(model,j,q) ;
        J = J + mass * body_COM_jac ;
    end
    J = 1/mass_sum * J ;
end