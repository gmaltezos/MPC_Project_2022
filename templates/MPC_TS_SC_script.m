% Script to provide the appropriate values for S and v
clear
clc

v = 0;
S = diag([0 0 0 0 0 0]);

params = generate_params();
Q = diag(params.exercise.QdiagOptA);
% Q = diag([50; 5; 100; 10;0;0]);
R = 3*eye(3);
N = params.model.HorizonLength;
[H, h] = lqr_maxPI(Q,R,params);
A = params.model.A;
B = params.model.B;

x = params.model.InitialConditionA;
obj1 = MPC_TS(Q,R,N,H,h,params);
u1(1:3,1:20) = 0;
for i = 1:20
    [u1(:,i), u_info1] = eval(obj1,x);
    x = A*x + B*u1(:,i);
end

y = params.model.InitialConditionA;
u2(1:3,1:20) = 0;
obj2 = MPC_TS_SC(Q,R,N,H,h,S,v,params);
for j=1:20
    [u2(:,j), u_info2] = eval(obj2,y);
    y = A*y + B*u2(:,i);
end

filename = 'MPC_TS_SC_params';
save(filename, "S", "v")