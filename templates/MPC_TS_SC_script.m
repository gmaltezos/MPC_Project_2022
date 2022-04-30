% Script to provide the appropriate values for S and v
v = 40;
S = 20;

params = generate_params();
Q = diag(params.exercise.QdiagOptA);
R = eye(3);
N = params.model.HorizonLength;
[H, h] = lqr_maxPI(Q,R,params);
x = params.model.InitialConditionA;
obj1 = MPC_TS(Q,R,N,H,h,params);
[u1, u_info1] = eval(obj1,x);

obj2 = MPC_TS_SC(Q,R,N,H,h,S,v,params);
[u2, u_info2] = eval(obj2,x);


filename = 'MPC_TS_SC_params';
save(filename, "S", "v")