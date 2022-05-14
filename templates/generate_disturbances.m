%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (c) 2022, Amon Lahr, Simon Muntwiler, Antoine Leeman & Fabian Flürenbrock Institute for Dynamic Systems and Control, ETH Zurich.
%
% All rights reserved.
%
% Please see the LICENSE file that has been included as part of this package.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function Wt = generate_disturbances(params)
    % YOUR CODE HERE
    % Parameter initialisation
    h_w = params.constraints.DisturbanceRHS;
    H_w = params.constraints.DisturbanceMatrix;
    Nt = params.model.HorizonLength;
    nx = params.model.nx;
%     Wt = rand(params_z.model.nx,Nt);

    % Uniform distributed disturbances
    a = h_w(2) / H_w(2);  % A MORE GENERAL IMPLEMENTATION COULD BE NEEDED
    b = h_w(1) / H_w(1);
    Wt = unifrnd(a,b,nx,Nt);
end