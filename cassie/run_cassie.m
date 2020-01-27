% Function to run Cassie Simulator
% This simulator is released for use by students of ME 193B / 292B.
% All other uses are not authorized.
% Copyright: Hybrid Robotics [hybrid-robotics.berkeley.edu], 2019.
function run_cassie()
    %% Initial Setup
    close all ;

    % Add various paths
%     addpath 'C:\Users\shawn\OneDrive\Documents\Berkeley\MEC231A\Lab4\tbxmanager';
%     tbxmanager restorepath; mpt_init;

    % Load Cassie model and set Initial configuration
    model = load('cassie_model.mat') ; model = model.model ;

    % Initial configuration
    x0 = getInitialState(model);

    % Perform any one-time setup and get student / control parameters
    [ctrl, student_data] = student_setup(x0, model);

    % ODE options
    time_inter = [0 5] ;
    odeopts = odeset('Events', @falldetect);
    externalForce_fun = @ExternalForce ;
    % Add any student-requested Integrators
    x0 = [x0; zeros(ctrl.N_integrators, 1)] ;


    %% Simulation 
    disp('Simulating...') ;
    tic

    if(ctrl.ode_type == 0)
        [t_vec, x_vec] = ode15s( @cassie_eom, time_inter, x0, odeopts, model, ctrl, externalForce_fun) ;
    else
        [t_vec, x_vec] =  ode45( @cassie_eom, time_inter, x0, odeopts, model, ctrl, externalForce_fun) ;
    end

    toc
    disp(['Simulated for ' num2str(t_vec(end)), 's'])

    %% Calculate Score
    score = calcScore(t_vec', x_vec', model);
    disp(['Score: ', num2str(score)])

    %%
    r_com = zeros(length(t_vec), 3) ;
    for i = 1 : size(x_vec,1)
        r_com(i,:) = compute_COM_pos(model, x_vec(i,1:model.NB))' ;
    end

    %% Calculate control signals
    disp('Computing control signals...') ;
    xdot_vec = zeros(size(x_vec)) ;
    tau_vec = zeros(length(t_vec), 20) ;
    for j=1:length(t_vec)
        [xdot_,tau_] = cassie_eom(t_vec(j),x_vec(j,:)', model,ctrl,externalForce_fun) ;
        xdot_vec(j,:) = xdot_' ;
        tau_vec(j,:) = tau_' ;
    end

    %% Plots and animation

    disp('Graphing...') ;
    % Plot COM position, base orientation, joint angles
    figure() ; 
        subplot(3,1,1);plot(t_vec, r_com) ;grid ; title('com positions x-y-z') ;hold; legend('x','y','z') ; ylabel('m') ;
        subplot(3,1,2); plot(t_vec, x_vec(:,4:6)*180/pi) ; grid ; title('base angles') ; ylabel('deg') ;
        subplot(3,1,3); plot(t_vec, x_vec(:,7:model.n)*180/pi) ; grid ; title('joint angles') ; ylabel('deg') ; xlabel('Time (s)') ;

    % Plot Base (Pelvis) Position
    figure ; plot(t_vec, x_vec(:,1:3)) ; grid on ;
        title('Base (Pelvis) Translation') ; legend('x','y','z') ; ylabel('m') ; xlabel('Time (s)') ;

    % Plot Base (Pelvis) Orientation
    figure ; plot(t_vec, x_vec(:,4:6)*180/pi) ; grid on ;
        title('Base (Pelvis) Orientation') ; legend('r','p','y') ; ylabel('deg') ; xlabel('Time (s)') ;

    % Plot Torques
    figure ; 
        subplot(2,1,1) ;
            plot(t_vec, tau_vec(:, [model.jidx.hip_abduction_left; model.jidx.hip_rotation_left; model.jidx.hip_flexion_left; model.jidx.knee_joint_left; model.jidx.toe_joint_left])) ;
            grid on ; title('Left Torques') ; legend('abduction','rotation','flexion','knee','toe') ; ylabel('Nm') ;
        subplot(2,1,2) ;
            plot(t_vec, tau_vec(:, [model.jidx.hip_abduction_right; model.jidx.hip_rotation_right; model.jidx.hip_flexion_right; model.jidx.knee_joint_right; model.jidx.toe_joint_right])) ;
            grid on ; title('Right Torques') ; legend('abduction','rotation','flexion','knee','toe') ; ylabel('Nm') ; xlabel('Time (s)') ;

    %% Animation
    stateData = getVisualizerState(x_vec, model);
    vis = CassieVisualizer(t_vec, stateData);
end