%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (c) 2022, Amon Lahr, Simon Muntwiler, Antoine Leeman & Fabian Flürenbrock Institute for Dynamic Systems and Control, ETH Zurich.
%
% All rights reserved.
%
% Please see the LICENSE file that has been included as part of this package.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function params = compute_tightening(K_tube,H_tube,h_tube,params)  
	% YOUR CODE HERE
    % Required polytopes
    E = Polyhedron(H_tube,h_tube);
    X = Polyhedron(params.constraints.StateMatrix, params.constraints.StateRHS);
    U = Polyhedron(params.constraints.InputMatrix,params.constraints.InputRHS);

    % Tightening constraints
    X = X - E;
    U = U - K_tube*E;

    % Ouputs
    params.constraints.StateMatrix = X.A;
    params.constraints.StateRHS = X.b;
    params.constraints.InputMatrix = U.A;
    params.constraints.InputRHS = U.b;
   
end