remove_idx = [
        model.idx.l_hand;
        model.idx.l_farm;
        model.idx.r_hand;
        model.idx.r_farm;
        model.idx.l_larm;
        model.idx.r_larm;
        model.idx.l_uarm;
        model.idx.r_uarm;
        model.idx.l_scap;
        model.idx.r_scap;
        model.idx.l_clav;
        model.idx.r_clav;

        model.idx.l_talus;
        model.idx.r_talus;
        model.idx.l_foot;
        model.idx.r_foot;

        model.idx.utorso;
        model.idx.mtorso;
        model.idx.ltorso;

        model.idx.head ;
];


q = rand(model.NB, 1) ;
dq = rand(model.NB, 1) ; dq(remove_idx) = 0 ;
ddq = rand(model.NB, 1) ; ddq(remove_idx) = 0 ;
r1 = compute_COM_pos(model, q) ;
T1 = ID(model, q, dq, ddq) ;


mm = combine_DOF(model, q, remove_idx) ;
qq = q ; qq(remove_idx) = [] ;
dqq = dq ; dqq(remove_idx) = [] ;
ddqq = ddq ; ddqq(remove_idx) = [] ;
r2 = compute_COM_pos(mm, qq) ;
T2 = ID(mm, qq, dqq, ddqq) ;
T2 = mm.map.tofull*T2 ; %); 0; T2(b:end)] ;
idx = setdiff(1:model.NB, remove_idx) ;

[r1 r2 r1-r2]
[T1(idx) T2(idx) T1(idx)-T2(idx)]
[q   mm.map.tofull*qq+mm.map.c_tofull   q-(mm.map.tofull*qq+mm.map.c_tofull)]






% % Test combine_DOF
% q = rand(model.NB, 1) ;
% b = model.idx.l_farm ;
% dq = rand(model.NB, 1) ; dq(b) = 0 ;
% ddq = rand(model.NB, 1) ; ddq(b) = 0 ;
% r1 = compute_COM_pos(model, q) ;
% T1 = ID(model, q, dq, ddq) ;
% 
% mm = combine_DOF(model, q, b) ;
% qq = q ; qq(b) = [] ;
% dqq = dq ; dqq(b) = [] ;
% ddqq = ddq ; ddqq(b) = [] ;
% r2 = compute_COM_pos(mm, qq) ;
% T2 = ID(mm, qq, dqq, ddqq) ;
% T2 = [T2(1:b-1); 0; T2(b:end)] ;
% 
% [r1 r2 r1-r2]
% [T1 T2 T1-T2]
% 



% pp = model.parent(p)
% 
% I_b = model.I{b} ;
% I_p = model.I{p} ;
% I_pp = model.I{pp} ;
% 
% X_b = mod_model.Xtree{b} ;
% X_p = mod_model.Xtree{p} ;
% 
% [m_b c_b] = mcI_inv(I_b) ;
% [m_p c_p] = mcI_inv(I_p) ;
% [m_pp c_pp] = mcI_inv(I_pp) ;
% 
% 
% [m c] = mcI_inv(I_p + X_b'*I_b*X_b)
% (m_p*c_p + m_b*(X_to_r(X_b)+c_b))/(m_p+m_b)
% 
% 
% [m c] = mcI_inv(I_pp + X_p'*(I_p + X_b'*I_b*X_b)*X_p)
% (m_pp*c_pp + m_p*(X_to_r(X_p)+c_p) +  m_b*(X_to_r(X_p)+X_to_r(X_b)+c_b))/(m_pp+m_p+m_b)
% 
% 
% 
% 
% model_orig = model ;
% for j=1:length(remove_idx)
%     model.jtype{j} = 'F' ;
% end