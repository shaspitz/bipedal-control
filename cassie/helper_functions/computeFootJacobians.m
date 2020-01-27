function [J1f, J1b, J2f, J2b] = computeFootJacobians(x,model)

q = x(1:20);
dq = x(21:40);

[qy, dqy, G] = model.gamma_q(model, q, dq) ;
 
[~, J1f] = bodyJac_vel(model, model.idx.foot1, q, xlt(model.p1)) ;
J1f = J1f*G;

[~, J1b] = bodyJac_vel(model, model.idx.foot1, q, xlt(model.p2)) ;
J1b = J1b*G;

[~, J2f] = bodyJac_vel(model, model.idx.foot2, q, xlt(model.p1)) ;
J2f = J2f*G;

[~, J2b] = bodyJac_vel(model, model.idx.foot2, q, xlt(model.p2)) ;
J2b = J2b*G;
