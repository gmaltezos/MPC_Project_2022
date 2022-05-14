%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (c) 2022, Amon Lahr, Simon Muntwiler, Antoine Leeman & Fabian Flürenbrock Institute for Dynamic Systems and Control, ETH Zurich.
%
% All rights reserved.
%
% Please see the LICENSE file that has been included as part of this package.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [H_tube,h_tube,n_iter] = compute_minRPI(K_tube,params)
    % YOUR CODE HERE
    % Parameter Initialisation
    conv = false;
    i=0;
%     params = generate_params(params);
    A = params.model.A;
    B = params.model.B;
    A_tube = A + B*K_tube;
    H_w = params.constraints.DisturbanceMatrix;
    h_w = params.constraints.DisturbanceRHS;
    W = Polyhedron(H_w,h_w);
    E1 = W;

    % Loop for computing the minRPI
    while conv == false
        i=i+1;
        E2 = E1 + (A_tube)^(i) * W;
        E2.minHRep();
        E1.minHRep();
        % Checking convergence
        if eq(E2,E1)
            conv = true;
        end
        E1 = E2;
    end

% Outputs
n_iter = i;
H_tube = E2.A;
h_tube = E2.b;
end