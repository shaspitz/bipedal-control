function [r_com, v_com] = computeComPosVel(q, dq, model)

r_com =  compute_COM_pos(model,q);
Jcom = COMJac_vel(model, q) ;
v_com = Jcom*dq ;
