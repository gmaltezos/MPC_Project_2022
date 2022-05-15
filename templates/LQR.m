%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (c) 2022, Amon Lahr, Simon Muntwiler, Antoine Leeman & Fabian Flürenbrock Institute for Dynamic Systems and Control, ETH Zurich.
%
% All rights reserved.
%
% Please see the LICENSE file that has been included as part of this package.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

classdef LQR
    properties
        K
    end
    
    methods
        %constructor
        function obj = LQR(Q,R,params)
            % YOUR CODE HERE
            % Solving RICATTI equation
            [F,~,~] = dlqr(params.model.A, params.model.B,Q,R);
%               [~,~,F] = dare(params.model.A,params.model.B,Q,R); 
%               [~,F,~] = idare(params.model.A,params.model.B,Q,R);

            obj.K = F; %(save feedback matrix for use in eval function)
        end
        % TODO Should it be called ctrl_info?
        function [u, u_info] = eval(obj,x)
            % YOUR CODE HERE
            u = - obj.K * x;
            u_info = struct('ctrl_feas',true);
        end
    end
end