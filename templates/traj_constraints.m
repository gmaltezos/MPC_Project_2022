%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (c) 2022, Amon Lahr, Simon Muntwiler, Antoine Leeman & Fabian Flürenbrock Institute for Dynamic Systems and Control, ETH Zurich.
%
% All rights reserved.
%
% Please see the LICENSE file that has been included as part of this package.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [s_max, y_max, u_max, J_u, df_max, vf_max, traj_feas] = traj_constraints(x,u,params)
    % YOUR CODE HERE
    
    % Maximum absolute value of x and z
    s_maxx = max(abs(x(1,:)));
    s_maxz = max(abs(x(3,:)));
    s_max = max(s_maxx, s_maxz);
    
    % Maximum absolute value of y
    y_max = max(abs(x(2,:)));
    
    % Maximum absolute value of applied thrust
    for i =1:size(u,2)
     u_norm = norm(u(:,i),"inf"); %% I am not sure for this calculation!!
    end
    u_max = max(u_norm);
    
    % Closed loop finite horizon input cost
    J_u = 0;
    for k = 1:size(u,2)
        J_u = J_u + u(:,k)'*u(:,k);
    end

%     Nt = params.model.HorizonLength;
    % Distance from the target position at Tf
    df_max = sqrt(x(1,end)^2 + x(2,end)^2 + x(3,end)^2);
    
    % Absolute differnece from the target velocity
    vf_max = sqrt(x(4,end)^2 + x(5,end)^2 + x(6,end)^2);
    
    %Boolean flag for feasibilty of the trajectory
    traj_feas = true;

    if s_max > params.constraints.MaxAbsPositionXZ || y_max > params.constraints.MaxAbsPositionY || u_max > params.constraints.MaxAbsThrust || df_max > params.constraints.MaxFinalPosDiff || vf_max > params.constraints.MaxFinalVelDiff
        traj_feas = false;
    end

end

