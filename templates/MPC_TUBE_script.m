% Script for task 31
clear all
clc

% Initialisation
R = 1;
N = 50;
p = [0.1 0.5];
Q = diag([20, 25]); % Need to be replaced probably with the values of qz,qvz found in the other script

% Design Tube controller
params = generate_params();
params_z = generate_params_z(params);
K_tube = compute_tube_controller(p,params_z);

% Compute the minimal robust positive invariant set
[H_tube,h_tube,n_iter] = compute_minRPI(K_tube,params_z);

% Obtain the tightened constraints
params_z_tube = params_z;
params_z_tube = compute_tightening(K_tube,H_tube,h_tube,params_z);

% Design the terminal set
[H_N, h_N] = lqr_maxPI(Q,R,params_z_tube);

% Save the necessary variables
filename = 'MPC_TUBE_params';
save(filename, "p", "K_tube", "H_tube", "h_tube", "H_N", "h_N", "params_z_tube")