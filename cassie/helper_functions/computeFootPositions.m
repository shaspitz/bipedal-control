function [p1, p2, p3, p4] = computeFootPositions(q, model)

X_foot1 = bodypos(model, model.idx.foot1, q) ;
X_foot2 = bodypos(model, model.idx.foot2, q) ;

p1 = X_to_r(xlt(model.p1)*X_foot1);
p2 = X_to_r(xlt(model.p2)*X_foot1);
p3 = X_to_r(xlt(model.p1)*X_foot2);
p4 = X_to_r(xlt(model.p2)*X_foot2);