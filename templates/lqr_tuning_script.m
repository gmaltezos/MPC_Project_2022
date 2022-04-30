% Tuning the lqr controller

%Initialisations
params = generate_params();
x0 = params.model.InitialConditionA;

% Tuning the dynamics in z direction
n = 10;
a = 0;
b = 2;
y = logspace(a,b,n);

Q = [0;0;2;0;0;4]; 
% for i=1:n
%     for j=1:n
%         if j ~= i
%         X = [0; 0; y(i); 0; 0; y(j)];
%         Q = [Q X];
%         end
%     end
% end

[tuning_struct, i_opt] = lqr_tuning(x0,Q,params)


% % Tuning the dynamics in xy directions
% y = logspace(a,b,n)
% 
% [tuning_struct, i_opt] = lqr_tuning(x0,Q,params)
% 
% 
% 
% filename = 'lqr_tuning_script.mat';
% save(filename, "tuning_struct", "q")
