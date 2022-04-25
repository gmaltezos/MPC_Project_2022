%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (c) 2022, Amon Lahr, Simon Muntwiler, Antoine Leeman & Fabian Flürenbrock Institute for Dynamic Systems and Control, ETH Zurich.
%
% All rights reserved.
%
% Please see the LICENSE file that has been included as part of this package.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [Xt,Ut,u_info] = simulate(x0, ctrl, params)

% YOUR CODE HERE
% Hint: you can access the control command with ctrl.eval(x(:,i))
    A = params.model.A;
    B = params.model.B;
    Nt = params.model.HorizonLength;
    Xt = zeros(size(x0,1), Nt+1);
    Xt(:,1) = x0;
    Ut = zeros(size(ctrl.K,1),Nt);

    for k = 2:Nt+1
%         [Ut(:,k-1), u_info(k-1)] = ctrl.eval(Xt(:,k-1));
        Ut(:,k-1) = ctrl.K * Xt(:,k-1);
        Xt(:,k) = A*Xt(:,k-1) + B*Ut(:,k-1);
    end
    u_info.ctrl_feas(1:Nt) = true;
% p = 2;
end