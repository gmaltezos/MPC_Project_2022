%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (c) 2022, Amon Lahr, Simon Muntwiler, Antoine Leeman & Fabian Flürenbrock Institute for Dynamic Systems and Control, ETH Zurich.
%
% All rights reserved.
%
% Please see the LICENSE file that has been included as part of this package.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [s_max, y_max, u_max, J_u, df_max, vf_max, traj_feas] = traj_constraints(x,u,params)
    % YOUR CODE HERE

    s_maxx = max(abs(x(1,:)));
    s_maxz = max(abs(x(3,:)));
    s_max = max(s_maxx, s_maxz);

    y_max = max(abs(x(2,:)));

    u_norm = norm(u,"inf"); %% I am not sure for this calculation!!
    u_max = max(u_norm);
    
    J_u = 0;
    for k = 1:size(u,2)
        J_u = J_u + u(:,k)'*u(:,k);
    end

%     Nt = params.model.HorizonLength;
    df_max = sqrt(x(1,end)^2 + x(2,end)^2 + x(3,end)^2);

    vf_max = sqrt(x(4,end)^2 + x(5,end)^2 + x(6,end)^2);

    traj_feas = true;

    if s_max > params.constraints.MaxAbsPositionXZ || y_max > params.constraints.MaxAbsPositionY || u_max > params.constraints.MaxAbsThrust || df_max > params.constraints.MaxFinalPosDiff || vf_max > params.constraints.MaxFinalVelDiff
        traj_feas = false;
    end

end

