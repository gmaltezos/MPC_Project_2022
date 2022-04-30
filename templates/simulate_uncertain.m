%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (c) 2022, Amon Lahr, Simon Muntwiler, Antoine Leeman & Fabian Flürenbrock Institute for Dynamic Systems and Control, ETH Zurich.
%
% All rights reserved.
%
% Please see the LICENSE file that has been included as part of this package.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [Xt,Ut,u_info] = simulate_uncertain(x0, ctrl, Wt, params_z)
	% YOUR CODE HERE
    A = params_z.model.A;
    B = params_z.model.B;
    Nt = params_z.model.HorizonLength;
    Xt = zeros(size(x0,1), Nt+1);
    Xt(:,1) = x0;
    Ut = zeros(size(ctrl.K,1),Nt);

    for k = 2:Nt+1
        dx = k-1;
        [Ut(:,k-1), u_info(dx)] = ctrl.eval(Xt(:,k-1));
        Xt(:,k) = A*Xt(:,k-1) + B*Ut(:,k-1) + Wt(:,k-1);
    end
end