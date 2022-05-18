%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (c) 2022, Amon Lahr, Simon Muntwiler, Antoine Leeman & Fabian Flürenbrock Institute for Dynamic Systems and Control, ETH Zurich.
%
% All rights reserved.
%
% Please see the LICENSE file that has been included as part of this package.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

classdef MPC_TUBE
    properties
        yalmip_optimizer
        K_tube
    end

    methods
        function obj = MPC_TUBE(Q,R,N,H_N,h_N,H_tube,h_tube,K_tube,params)
            obj.K_tube = K_tube;

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
            V = sdpvar(repmat(nu,1,N),ones(1,N),'full');
            X0 = sdpvar(nx,1,'full');
            Z = sdpvar(repmat(nx,1,N+1),ones(1,N+1),'full');
            
            % DEfine objective and constraints for tube mpc
            objective = 0;
            constraints = [H_tube * (X0 - Z{1}) <= h_tube];

             for k = 1:N
                constraints = [ ...
                    constraints, ...
                    Z{k+1} == A*Z{k} + B*V{k} , ...
                    H_x * Z{k} <= h_x, ...
                    H_u * V{k} <= h_u ...
                ];

                objective = objective + Z{k}'*Q*Z{k} + V{k}'*R*V{k};
             end

            % terminal constraint
            constraints = [ ...
                constraints, ...
                H_N * Z{N+1} <= h_N
            ];

            %Terminal cost
            [~,P,~] = dlqr(A, B, Q, R);
            J_Nt = Z{N+1}' * P * Z{N+1};
            objective = objective + J_Nt;
            
            % Define optimizer
            opts = sdpsettings('verbose',1,'solver','quadprog');
            obj.yalmip_optimizer = optimizer(constraints,objective,opts,X0,{V{1} Z{1} objective});
        end

        function [u, u_info] = eval(obj,x)
            %% evaluate control action by solving MPC problem, e.g.
            tic;
            [optimizer_out,errorcode] = obj.yalmip_optimizer(x);
            solvetime = toc;
            % YOUR CODE HERE
            % Control law for Tube MPC
            [v, z, objective] = optimizer_out{:};
            u = v + obj.K_tube * (x - z);

            feasible = true;
            if (errorcode ~= 0)
                feasible = false;
            else
                u = v + obj.K_tube * (x - z);
            end

            u_info = struct('ctrl_feas',feasible,'objective',objective,'solvetime',solvetime);
        end
    end
end