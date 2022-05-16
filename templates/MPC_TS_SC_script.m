% Script to provide the appropriate values for S and v
clear
clc

v = 0;
S = diag([0 0 0 0 0 0]);

params = generate_params();
% Q = diag(params.exercise.QdiagOptA);
Q = diag([150; 150; 150; 10;0;0]);
R = 3*eye(3);
N = 200;
[H, h] = lqr_maxPI(Q,R,params);
A = params.model.A;
B = params.model.B;

x = params.model.InitialConditionB;
%params.constraints.StateMatrix=zeros(6,6);
%params.constraints.StateRHS=zeros(6,1);
obj1 = MPC_TS(Q,R,N,H,h,params);
u1 = zeros(3, 20);
for i = 1:20
    [u1(:,i), u_info1] = eval(obj1,x);
    x = A*x + B*u1(:,i);
    u_info1
end

y = params.model.InitialConditionB;
u2 = zeros(3, 20);
obj2 = MPC_TS_SC(Q,R,N,H,h,S,v,params);
for j=1:20
    [u2(:,j), u_info2] = eval(obj2,y);
    y = A*y + B*u2(:,j);
    u_info2
end

filename = 'MPC_TS_SC_params';
save(filename, "S", "v")