% Function that computes the time-derivative of states of Cassie
function [xdot, tau] = cassie_eom(t,s, model, params, externalForce)

    %% Extract generalized coordinates and velocities
    q = s(1 : model.n);
    dq = s(model.n+1 : 2*model.n);


    %% Compute Student Control input
    % get STUDENT Control
    tauStudent = studentController(t, s, model, params);

    %% Some pre-processing of the torques before we simulate
    % Add torque saturation
    tauMax = repmat([25*4.5; 25*4.5; 16*12.2; 16*12.2; 50*0.9], 2, 1);
    tauStudent = min(max(tauStudent, -tauMax), tauMax);
    % Set torques for the actuated coordinates
    tau  = zeros(20,1);
    tau(model.actuated_idx) = tauStudent;

    %% ----------------------------External forces-------------------------------
    % -------------------------------------------------------------------------
    % f_ext{i} must either be an empty array, indicating
    % that there is no external force acting on body i, or else a spatial or
    % planar vector (as appropriate) giving the external force expressed in
    % absolute coordinates.
    f_ext = externalForce(t, q, model);

    %% ----------------------------Dynamics-------------------------------------
    % -------------------------------------------------------------------------
    % ground contact model
    K = 1e6 ;
    D = 2000 ;
    mu = 0.8 ;

    % ground contact states
    gc_d = reshape(s((2*model.n+1) : (2*model.n + 2*length(model.gc.body))), 2, length(model.gc.body)) ;

    % compute ground contact force
    posvel = gcPosVel(model, q, dq) ;
    [fp, gc_dd, ~] = gcontact( K, D, mu, posvel(1:3,:), posvel(4:6,:), gc_d) ;
    f = Fpt(fp, posvel(1:3,:)) ;

    % compute accelerations
    ddq = gcFD_pert(model, @FDgq, [q; dq; tau; reshape(f, 6*length(model.gc.body), 1)],f_ext) ;

    % state derivative
    xdot = [dq; 
            ddq; 
            reshape(gc_dd, 2*length(model.gc.body), 1)] ;

end