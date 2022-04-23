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
    A = params.model.A;
    B = params.model.B;
    [F,~,~] = dlqr(A, B, Q, R);
    K = -F;

    % computes a control invariant set for LTI system x^+ = A*x+B*u
    system = LTISystem('A', A + B*K);
    
    % The constraints should not be hardcoded, because in every test
    % scenario, they provide a different system.
    nx = params.model.nx;
    nu = params.model.nu;
    H_u = params.constraints.InputMatrix;
    h_u = params.constraints.InputRHS;
    H_x = params.constraints.StateMatrix;
    h_x = params.constraints.StateRHS;
    xMin = [];
    xMax = [];
    uMin = [];
    uMax = [];
    for i =1:2:2*nx
        xMin = [xMin; -h_x(i+1)];
        xMax = [xMax; h_x(i)];
    end
    for j=1:2:2*nu
        uMin = [uMin; -h_u(j+1)];
        uMax = [uMax; h_u(j)];
    end
    system.x.min = xMin;
    system.x.max = xMax;
    system.u.min = uMin;
    system.u.max = uMax;
    InvSet = system.invariantSet();
    InvSet.plot()
    H = InvSet.A;
    h = InvSet.b;
    stopp3 = 1;
end

