function stateData = getVisualizerState(qData, model)
% Get the state information required by the visualizer

% Ayush Agrawal
qData = qData';

pos = qData([model.jidx.pelvis_x,model.jidx.pelvis_y,model.jidx.pelvis_z], :);

quat = [];

for i = 1:size(qData,2)
    r = qData(model.jidx.pelvis_Rx, i);
    p = qData(model.jidx.pelvis_Ry, i);
    y = qData(model.jidx.pelvis_Rz, i);
    rotm = rot_x(r)*rot_y(p)*rot_z(y);
    rotm = Rotation3d(rotm);
    q = rotm.getQuaternion;
    q = q.getValue;
    quat = [quat, q];
end

qm = qData([model.jidx.hip_abduction_left, 
            model.jidx.hip_rotation_left, 
            model.jidx.hip_flexion_left, 
            model.jidx.knee_joint_left, 
            model.jidx.toe_joint_left, 
            model.jidx.hip_abduction_right, 
            model.jidx.hip_rotation_right, 
            model.jidx.hip_flexion_right, 
            model.jidx.knee_joint_right, 
            model.jidx.toe_joint_right], :);
        
qj = [qData(model.jidx.knee_to_shin_left, :);
      qData(model.jidx.ankle_joint_left, :);
      qData(model.jidx.toe_joint_left, :);
      qData(model.jidx.knee_to_shin_right, :);
      qData(model.jidx.ankle_joint_right, :);
      qData(model.jidx.toe_joint_right, :)];
  
stateData = [quat;
             pos;
             qm;
             qj];

