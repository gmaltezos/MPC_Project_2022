%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (c) 2022, Amon Lahr, Simon Muntwiler, Antoine Leeman & Fabian Flürenbrock Institute for Dynamic Systems and Control, ETH Zurich.
%
% All rights reserved.
%
% Please see the LICENSE file that has been included as part of this package.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

classdef MPC_TE_forces
    properties
        forces_optimizer
    end

    methods
        function obj = MPC_TE_forces(Q,R,N,params)
            % YOUR CODE HERE
            % Parameter Initialisation
            A = params.model.A;
            B = params.model.B;

            nu = params.model.nu;
            nx = params.model.nx;

            H_x = params.constraints.StateMatrix;
            h_x = params.constraints.StateRHS;
            H_u = params.constraints.InputMatrix;
            h_u = params.constraints.InputRHS;

            % define optimization variables
            U = sdpvar(repmat(nu,1,N),ones(1,N),'full');
            X0 = sdpvar(nx,1,'full');
            X = sdpvar(repmat(nx,1,N+1),ones(1,N+1),'full');
            
            % Define objective and constraints
            objective = 0;
            constraints = X{1} == X0;
            for k = 1:N
                constraints = [ ...
                    constraints, ...
                    X{k+1} == A*X{k} + B*U{k} , ...
                    H_x * X{k} <= h_x, ...
                    H_u * U{k} <= h_u ...
                ];

                objective = objective + X{k}'*Q*X{k} + U{k}'*R*U{k};
            end

            % terminal constraint
            constraints = [ ...
                constraints, ...
                X{N+1} == zeros(nx,1)
            ];

            % Define optimizer (using FORCES)
            opts = getOptions('forcesSolver');
            opts.printlevel = 0;
            obj.forces_optimizer = optimizerFORCES(constraints,objective,opts,X0,{U{1}});% YOUR CODE HERE
        end

        function [u, u_info] = eval(obj,x)
            %% evaluate control action by solving MPC problem, e.g.
            [optimizer_out,errorcode,info] = obj.forces_optimizer(x);
            u = optimizer_out;
            objective = info.pobj;
            solvetime = info.solvetime;

            feasible = true;
            if any(errorcode ~= 1)
                feasible = false;
                warning('MPC infeasible');
            end

            u_info = struct('ctrl_feas',feasible,'objective',objective,'solvetime',solvetime);
        end
    end
end