%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (c) 2022, Amon Lahr, Simon Muntwiler, Antoine Leeman & Fabian Flürenbrock Institute for Dynamic Systems and Control, ETH Zurich.
%
% All rights reserved.
%
% Please see the LICENSE file that has been included as part of this package.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [H_u, h_u, H_x, h_x] = generate_constraints(params)
    % YOUR CODE HERE
    % Generation of polytopic constraints for state and input
    umax = params.constraints.MaxAbsThrust;
    h_u = repmat([umax;umax], params.model.nu,1);
    H_u = zeros(size(h_u, 1), params.model.nu);
    H_u(1:2:end,:) = eye(params.model.nu);
    H_u(2:2:end,:) = -eye(params.model.nu);
    %h_u = [umax; umax; umax; umax; umax; umax];
    smax = params.constraints.MaxAbsPositionXZ;
    ymax = params.constraints.MaxAbsPositionY;
    % TODO: Replace hardcoded matrices
    H_x = [1 0 0 0 0 0; -1 0 0 0 0 0; 0 1 0 0 0 0; 0 -1 0 0 0 0; 0 0 1 0 0 0; 0 0 -1 0 0 0];
    % TODO Constraint matrices must be scaled
    h_x = [smax; smax; ymax; ymax; smax; smax];
end