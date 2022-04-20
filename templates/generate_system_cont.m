%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (c) 2022, Amon Lahr, Simon Muntwiler, Antoine Leeman & Fabian Flürenbrock Institute for Dynamic Systems and Control, ETH Zurich.
%
% All rights reserved.
%
% Please see the LICENSE file that has been included as part of this package.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [Ac, Bc] = generate_system_cont(params)
    % YOUR CODE HERE
    wn = sqrt(params.model.GravitationalParameter/(params.model.TargetRadius^3));
    Ac = [0 0 0 1 0 0; 0 0 0 0 1 0; 0 0 0 0 0 1; 3*wn^2 0 0 0 2*wn 0; 0 0 0 -2*wn 0 0; 0 0 -wn^2 0 0 0];
    Bc = [0 0 0; 0 0 0; 0 0 0; 1/params.model.Mass 0 0; 0 1/params.model.Mass 0; 0 0 1/params.model.Mass];
end