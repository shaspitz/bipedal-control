% Function to compute the joint positions
function joint_pos = compute_joint_pos(model, q)
    XXup{1} = eye(6) ;
    P0 = [0;0;0;  0;0;0] ;
    for i = 1:model.NB
      [ XJ, S{i} ] = jcalc( model.jtype{i}, q(i) );
      Xup{i} = XJ * model.Xtree{i};
      if(i > 1)
        XXup{i} = Xup{i}*XXup{model.parent(i)} ;
      else
        XXup{i} = Xup{i} ;
      end
      joint_pos(:,i) = X_to_r(XXup{i}) ;
    end
    if(length(model.Xtree) > model.NB)
        joint_pos(:,i+1) = X_to_r(model.Xtree{i+1}*XXup{i}) ;
    end
end