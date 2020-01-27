function [q dq G gs] = gamma_q_constraint_spring_4bar(model, q0, dq0)
  dependent_idx = model.dependent_idx ;
  independent_idx = model.independent_idx ;
  
% y = formula for calculating y from qo;
  y = q0(independent_idx) ;

% q = formula for gamma(y);
  q = q0 ;
  q(independent_idx) = q0(independent_idx) ;
  q(dependent_idx) = [0;
                      0;
                      deg2rad(13)-q0(model.jidx.knee_joint_left);
                      deg2rad(13)-q0(model.jidx.knee_joint_right)] ;

% G = Jacobian of gamma;
  G = zeros(model.NB, length(y)) ;
  for j=1:length(y)
      G(independent_idx(j), j) = 1 ;
  end
  G(dependent_idx(3), find(independent_idx == model.jidx.knee_joint_left)) = -1 ;
  G(dependent_idx(4), find(independent_idx == model.jidx.knee_joint_right)) = -1 ;
  
%   [q  G*y  q-G*y]

% dy = formula for calculating dy from qo (or y) and dqo;
  dy = dq0(independent_idx) ;
  dq = G * dy ;

% g = formula for dG/dt * yd;
  g = zeros(model.NB, 1) ;

% Tstab = some suitable value, such as 0.1;
  Tstab = 0.1 ;
  gstab = 2/Tstab * (dq - dq0) + 1/Tstab^2 * (q - q0);
  gs = g + gstab;
end