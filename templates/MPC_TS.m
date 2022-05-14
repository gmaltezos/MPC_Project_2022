%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (c) 2022, Amon Lahr, Simon Muntwiler, Antoine Leeman & Fabian Flürenbrock Institute for Dynamic Systems and Control, ETH Zurich.
%
% All rights reserved.
%
% Please see the LICENSE file that has been included as part of this package.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

classdef MPC_TS
    properties
        yalmip_optimizer
    end

    methods
        function obj = MPC_TS(Q,R,N,H,h,params)
            % YOUR CODE HERE
            % Parameter initialisation 
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
            % Terminal constraint
            constraints = [ ...
                constraints, ...
                H * X{N+1} <= h
            ];

            %Terminal cost
            [~,P,~] = dlqr(A, B, Q, R);
            J_Nt = X{N+1}' * P * X{N+1};
            objective = objective + J_Nt;
            
            % Define the optimizer
            opts = sdpsettings('verbose',1,'solver','quadprog');
            obj.yalmip_optimizer = optimizer(constraints,objective,opts,X0,{U{1} objective});
        end

        function [u, u_info] = eval(obj,x)
            %% evaluate control action by solving MPC problem, e.g.
            tic;
            [optimizer_out,errorcode] = obj.yalmip_optimizer(x);
            solvetime = toc;
            [u, objective] = optimizer_out{:};

            feasible = true;
            if (errorcode ~= 0)
                feasible = false;
            end

            u_info = struct('ctrl_feas',feasible,'objective',objective,'solvetime',solvetime);
        end
    end
end