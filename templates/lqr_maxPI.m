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
    xMin = [];
    xMax = [];
    uMin = [];
    uMax = [];
    for i =1:2:size(h_x,1)
        if any(H_x(i,:),[2])
            xMin = [xMin; -h_x(i+1)];
            xMax = [xMax; h_x(i)];
        end
    end
    for j=1:2:size(h_u,1)
        if any(H_u(j,:),[2])
            uMin = [uMin; -h_u(j+1)];
            uMax = [uMax; h_u(j)];
        end
    end
    system.x.min = [xMin; zeros(nx-size(xMin,1),1)];
    system.x.max = [xMax; zeros(nx-size(xMax,1),1)];
    %system.u.min = uMin;
    %system.u.max = uMax;
    %controller.model.x.with(’terminalSet’)
    %controller.model.x.terminalSet = T
    %P1 = Polyhedron('lb', uMin, 'ub', uMax);
    % TODO: Shouldn't the terminal set be part of the construction of the
    % invariant set too?

    b = [uMax; uMax];
    A = [];
    while size(A,1) ~= size(b,1)
        A = [A; -K; K];
    end
%     A = [-K; K];
    P = Polyhedron (A, b);
    system.setDomain('x', P);
    %%controller.model.x.with(’setConstraint’)
    %controller.model.x.setConstraint = X
    InvSet = system.invariantSet();
%     InvSet.plot()
    H = InvSet.A;
    h = InvSet.b;
end

