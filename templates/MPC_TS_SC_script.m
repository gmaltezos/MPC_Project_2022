% Script to provide the appropriate values for S and v
clear
clc

v = 10000;
S = diag([100 100 100 100 100 100]);

params = generate_params();
% Q = diag(params.exercise.QdiagOptA);
Q = diag([150; 150; 150; 10;0;0]);
R = 3*eye(params.model.nu);
N = 30;
[H, h] = lqr_maxPI(Q,R,params);
A = params.model.A;
B = params.model.B;

x = params.model.InitialConditionA;
%params.constraints.StateMatrix=zeros(6,6);
%params.constraints.StateRHS=zeros(6,1);
obj1 = MPC_TS(Q,R,N,H,h,params);
u1 = zeros(params.model.nu, 20);
for i = 1:20
    [u1(:,i), u_info1] = eval(obj1,x);
    x = A*x + B*u1(:,i);
    if ~u_info1.ctrl_feas
        msg = strcat('MPC_TS was infeasible at', string(i));
        error(msg);          
    end
end

y = params.model.InitialConditionA;
u2 = zeros(params.model.nu, 20);
obj2 = MPC_TS_SC(Q,R,N,H,h,S,v,params);
for j=1:20
    [u2(:,j), u_info2] = eval(obj2,y);
    y = A*y + B*u2(:,j);
    if ~u_info2.ctrl_feas
        msg = strcat('MPC_TS_SC was infeasible at', string(i));
        error(msg);          
    end
end

inputsSameCheck = isequal(round(u1,16), round(u1,16))
filename = 'MPC_TS_SC_params';
save(filename, "S", "v")