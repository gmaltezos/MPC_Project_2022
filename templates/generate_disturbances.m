%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (c) 2022, Amon Lahr, Simon Muntwiler, Antoine Leeman & Fabian Flürenbrock Institute for Dynamic Systems and Control, ETH Zurich.
%
% All rights reserved.
%
% Please see the LICENSE file that has been included as part of this package.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function Wt = generate_disturbances(params_z)
    % YOUR CODE HERE
    h_w = params_z.constraints.DisturbanceRHS;
    H_w = params_z.constraints.DisturbanceMatrix;
    Nt = params_z.model.HorizonLength;
    nx = params_z.model.nx;
%     Wt = rand(params_z.model.nx,Nt);
    a = h_w(2) / H_w(2);  % A MORE GENERAL IMPLEMENTATION COULD BE NEEDED
    b = h_w(1) / H_w(1);
    Wt = unifrnd(a,b,nx,Nt);
end