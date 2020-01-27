% floatbase_XYZ takes a base that is not fixed (having 3 revolte DOF) and
% frees the translation 3DOF.  This is done by introducing 3 new
% translation joints and by preserving everything else.
% NOTE The original model has the base NOT fixed with the revolte DOF
% floating already.
function  fbmodel = floatbase_XYZ( model )


if size(model.Xtree{1},1) == 3
  error('floatbase applies to spatial models only');
end

if any( model.parent(2:model.NB) == 0 )
  error('only one connection to a fixed base allowed');
end

if isfield( model, 'gamma_q' )
  warning('floating a model with gamma_q (joint numbers will change)');
end

if ~isequal( model.Xtree{1}, eye(6) )
  warning('Xtree{1} not identity');
end

fbmodel = model;

fbmodel.NB = model.NB + 3;

fbmodel.jtype = ['Px' 'Py' 'Pz' model.jtype(1:end)];

fbmodel.parent = [0 1 2 model.parent+3];

fbmodel.Xtree = [eye(6) eye(6) eye(6) model.Xtree];

fbmodel.I = [zeros(6) zeros(6) zeros(6) model.I];

if isfield( model, 'appearance' )
  fbmodel.appearance.body = {{}, {}, {}, model.appearance.body{:}};
end

if isfield( model, 'camera' ) && isfield( model.camera, 'body' )
  if model.camera.body > 0
    fbmodel.camera.body = model.camera.body + 3;
  end
end

if isfield( model, 'gc' )
  fbmodel.gc.body = model.gc.body + 3;
end

end
