% Function to setup and perform any one-time computations
% and / or define parameters used by the controller
% Input
%   x0 - Initial state
%   model - struct containing robot properties
% Output
%   ctrl - struct with various user-defined members
%   ctrl.ode_type - change this to change between ode15s (default) and ode45 
%                   default choice is typically fine in most cases
%   ctrl.N_integrators - change this to add any integrators.
%
%   student_data - struct with members id and team_name
%   student_data.id - Enter your student id
%   student_data.team_name - Enter your team name
function [ctrl, student_data] = student_setup(x0, model)


    %% Set Student Data  [UPDATE THIS TO SUBMIT]
    % Provide student information.  Enter your student id and your team name.
    % Your student id will be used to register your score for your grade.
    % Your team_name will be displayed on the leaderboard - this will serve to anonymize you to your peers.  (Choose a wise / cheeky nick name.)
    % Ensure all members of your team use the same team name.
    % You can change your team name over the course of submissions if needed.
    student_data.id = 3035296317 ;
    student_data.team_name = 'codiecode' ;

    
    %% Set any Control Parameters
    % ode_type and N_integrators are required parameters.
    % You can add any other parameters by doing a one-time computation here.
    % These parameters are then available in your controller file.
    
    % We provide possibility to switch the ode solver if needed.
    % Default ode_type should work in almost all cases.  Don't change this unless you know what you are doing.
    ctrl.ode_type = 1 ; % zero => ode15s, non-zero=> ode45.
    
    % No. of integrators.  These integrators will be added to the ode states.
    % Your student_controller file will have to supply the derivative (dx)
    % of these additional states.  The Ode solver will then integrate this for you.
    % In most cases, you do NOT need any integrator states
    ctrl.N_integrators = 0 ;
end
