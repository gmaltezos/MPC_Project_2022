%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (c) 2022, Amon Lahr, Simon Muntwiler, Antoine Leeman & Fabian Flürenbrock Institute for Dynamic Systems and Control, ETH Zurich.
%
% All rights reserved.
%
% Please see the LICENSE file that has been included as part of this package.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% BRIEF:
%   Template for explicit invariant set computation. You MUST NOT change
%   the output.
% INPUT:
%   Q, R: State and input weighting matrix
% OUTPUT:
%   H, h: Describes polytopic X_LQR = {x | H * x <= h}

function [H, h] = lqr_maxPI(Q,R,params)
	% YOUR CODE HERE
    % Parameter initialisation
    A = params.model.A;
    B = params.model.B;

    % Gain of LQR controller
    [F,~,~] = dlqr(A, B, Q, R);
    K = F;

    % Resulting LTI system with the lqr controller
    system = LTISystem('A', A - B*K);
    
    % The constraints should not be hardcoded, because in every test
    % scenario, they provide a different system.
    nx = params.model.nx;
    nu = params.model.nu;
    H_u = params.constraints.InputMatrix;
    h_u = params.constraints.InputRHS;
    H_x = params.constraints.StateMatrix;
    h_x = params.constraints.StateRHS;
    Xp = Polyhedron('A',[H_x; - H_u * K], 'b', [h_x; h_u]);
    system.setDomain('x', Xp);
    InvSet = system.invariantSet();
    H = InvSet.A;
    h = InvSet.b;
end

