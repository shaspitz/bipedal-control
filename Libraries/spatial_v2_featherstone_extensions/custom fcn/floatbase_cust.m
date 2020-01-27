function  fbmodel = floatbase_cust( model, joints )

% floatbase  construct the floating-base equivalent of a fixed-base model
% floatbase(model)  converts a fixed-base spatial kinematic tree to a
% floating-base kinematic tree as follows: old body 1 becomes new body 6,
% and is regarded as the floating base; old joint 1 is discarded; six new
% joints are added (three prismatic and three revolute, in that order,
% arranged along the x, y and z axes, in that order); and five new
% zero-mass bodies are added (numbered 1 to 5), to connect the new joints.
% All other bodies and joints in the given model are preserved, but their
% identification numbers are incremented by 5.  The end result is that body
% 6 has a full 6 degrees of motion freedom relative to the fixed base, with
% joint variables 1 to 6 serving as a set of 3 Cartesian coordinates and 3
% rotation angles (about x, y and z axes) specifying the position and
% orientation of body 6's coordinate frame relative to the base coordinate
% frame.  CAUTION: A singularity occurs when q(5)=+-pi/2.  If this is a
% problem then use the special functions FDfb and IDfb instead of the
% standard dynamics functions.  floatbase requires that old joint 1 is the
% only joint connected to the fixed base, and that Xtree{1} is the
% identity.

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

N_joints = length(joints) ;

fbmodel.NB = model.NB + (N_joints-1);

fbmodel.jtype = [joints{:} model.jtype(2:end)];

fbmodel.parent = [0:(N_joints-2)   model.parent+(N_joints-1)];

for j=1:N_joints-1
    Xtree{j} = eye(6) ;
    I{j} = zeros(6) ;
    body{j} = {} ;
end

fbmodel.Xtree = [Xtree model.Xtree];

fbmodel.I = [I model.I];

if isfield( model, 'appearance' )
  fbmodel.appearance.body = {body{:} model.appearance.body{:}};
end

if isfield( model, 'camera' ) && isfield( model.camera, 'body' )
  if model.camera.body > 0
    fbmodel.camera.body = model.camera.body + (N_joints-1);
  end
end

if isfield( model, 'gc' )
  fbmodel.gc.body = model.gc.body + (N_joints-1);
end
