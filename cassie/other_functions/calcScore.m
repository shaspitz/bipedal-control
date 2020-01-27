function J = calcScore(t, s, model)

q = s(1 : model.n, :);
dq = s(model.n+1 : 2*model.n, :);
v = dq(1:3, :);
w = dq(4:6, :);

vNorm = vecnorm(v);
wNorm = vecnorm(w);

vInfo = lsiminfo(vNorm, t);
wInfo = lsiminfo(wNorm, t);
tSettle = vInfo.SettlingTime;
vNormEnd = norm(v(:, end));
wNormEnd = norm(w(:, end));

vMax = 5;
wMax = 1;
tMax = 5;



if t < 5
    J = 0; % Robot has fallen down
else
    J = 100*(max(0,(vMax-vNormEnd)) + max(0, (wMax - wNormEnd)) + (tMax - tSettle))/(vMax + wMax + tMax);
end


