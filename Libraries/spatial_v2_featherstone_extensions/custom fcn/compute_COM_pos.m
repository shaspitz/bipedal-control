% Function to compute the COM positions
function [COM_pos body_COM_pos] = compute_COM_pos(model, q)
    body_COM_pos = zeros(3, model.NB) ;
    if(isa(q, 'sym'))
        body_COM_pos = sym(body_COM_pos) ;
    end
	mass_sum = 0;
	COM_pos = zeros(3,1) ;
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
      joint_pos = X_to_r(XXup{i}) ;
      
      I = model.I{i} ;
      mass(i) = I(6,6) ;
      if(mass(i) > 0)
        com_offset = [I(3,5) ;
                      I(1,6) ;
                      I(2,4)] / mass(i) ;

        X_com = xlt(com_offset)*XXup{i} ;
        body_COM_pos(:,i) = X_to_r(X_com) ;
        
        mass_sum = mass_sum + mass(i) ;
        COM_pos = COM_pos + mass(i) * body_COM_pos(:,i) ;
      else
          body_COM_pos(:,i) = zeros(3,1) ;
      end
    end
    COM_pos = COM_pos / mass_sum ;
end