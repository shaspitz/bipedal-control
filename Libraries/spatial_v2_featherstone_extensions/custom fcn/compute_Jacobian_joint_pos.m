% Function to compute the jacobian of the joint positions wrt to joint
% angles dp_k / dq, p_k = position of kth joint location.
function J = compute_Jacobian_joint_pos(model, q, body_n)
    XXup{1} = eye(6) ;
    eb = zeros(1, model.NB) ;
    eb(model.kappa{body_n}) = 1 ;
    
    J = zeros(6, model.NB) ;
    for i = 1:model.NB
      [ XJ, S ] = jcalc( model.jtype{i}, q(i) );
      Xup{i} = XJ * model.Xtree{i};
      if(i > 1)
        XXup{i} = Xup{i}*XXup{model.parent(i)} ;
      else
        XXup{i} = Xup{i} ;
      end
      if(eb(i))
        J(:,i) = XXup{i} \ S ;  % Eqn(20),(21),(25) in tutorial #2
      end
    end
end