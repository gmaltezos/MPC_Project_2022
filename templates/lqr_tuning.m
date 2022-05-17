%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (c) 2022, Amon Lahr, Simon Muntwiler, Antoine Leeman & Fabian Flürenbrock Institute for Dynamic Systems and Control, ETH Zurich.
%
% All rights reserved.
%
% Please see the LICENSE file that has been included as part of this package.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [tuning_struct, i_opt] = lqr_tuning(x0,Q,params)
    % YOUR CODE HERE
    % Loop over different Qs for different LQR controllers
    i_opt = nan;
    % iterate through parameter vectors
    for i = 1:size(Q,2)
        % Calculating LQR controller, simulating the resulting trajectory
        % and obtaining the trajectory constraints
        R = eye(params.model.nu);
        obj = LQR(diag(Q(:,i)),R,params);
        [x,u,~] = simulate(x0, obj, params);
        [s_max, y_max, u_max, J_u, df_max, vf_max, traj_feas] = traj_constraints(x,u,params);

        % Constructing the struct required for the lqr tuning
        tuning_struct(i,:) = struct(...
        'InitialCondition', x0, ...
        'Qdiag', Q(:,i), ...
        'MaxAbsPositionXZ', s_max, ...
        'MaxAbsPositionY', y_max, ...
        'MaxAbsThrust', u_max, ...
        'InputCost', J_u, ...
        'MaxFinalPosDiff', df_max, ... 
        'MaxFinalVelDiff', vf_max, ... 
        'TrajFeasible', traj_feas ...
    );
         % Check the feasibility of the trajectory and find the the best
         % lqr controller
        if isnan(i_opt) & (tuning_struct(i).TrajFeasible == true) 
            i_opt = i;
        elseif ~isnan(i_opt) & (tuning_struct(i).TrajFeasible) == true & (tuning_struct(i).InputCost < tuning_struct(i_opt).InputCost)
            i_opt = i;
        end
    end
  end