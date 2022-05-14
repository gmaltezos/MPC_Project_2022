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
    H_u = [1 0 0; -1 0 0; 0 1 0; 0 -1 0; 0 0 1; 0 0 -1];
    umax = params.constraints.MaxAbsThrust;
    h_u = [umax; umax; umax; umax; umax; umax];
    smax = params.constraints.MaxAbsPositionXZ;
    ymax = params.constraints.MaxAbsPositionY;
    % TODO: Replace hardcoded matrices
    H_x = [1 0 0 0 0 0; -1 0 0 0 0 0; 0 1 0 0 0 0; 0 -1 0 0 0 0; 0 0 1 0 0 0; 0 0 -1 0 0 0];
    % TODO Constraint matrices must be scaled
    h_x = [smax; smax; ymax; ymax; smax; smax];
end