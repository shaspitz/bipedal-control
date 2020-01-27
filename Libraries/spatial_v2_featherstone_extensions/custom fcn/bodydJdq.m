%This function computes dJ/dt dq/dt - Second term of Eq (32) in Part II of
%Featherstone paper.  It's based on the computations in ID.m converted to
%recursive form.
% Input Parameters:
%  model - The model
%  q - joint position
%  dq - joint velocity
%  b - index of body
function dJdq_ = dJdq(model, q, qd, b)
    [~, dJdq_] = dJdq_internal(model, q, qd, b) ;
end


% Internal recursive function
function [v_i, a_i] = dJdq_internal(model, q, qd, i)
  p = model.parent(i) ;
  
  [ XJ, S ] = jcalc( model.jtype{i}, q(i) );
  vJ = S*qd(i);
  Xup = XJ * model.Xtree{i};

  if p == 0
    v_i = vJ;
    a_grav = get_gravity(model);
    a_i = Xup*(-a_grav) ;
  else
    [v_p, a_p] = dJdq_internal(model, q, qd, p) ;
    v_i = Xup*v_p + vJ;
    a_i = Xup*a_p + crm(v_i)*vJ;
  end
end