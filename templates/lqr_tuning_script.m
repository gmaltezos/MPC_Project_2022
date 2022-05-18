% Tuning the lqr controller
% By first tuning in the z direction (first searching in the log-space and
% then in the linear space) and then in the xy directions with the same
% idea
clear
clc

%Initialisations
params = generate_params();
x0 = params.model.InitialConditionA;

% % Tuning the dynamics in z direction
% Searching in the log space
% n = 100;
% ay = 2;
% by = 3;
% az = 0;
% bz = 1;
% y = logspace(ay,by,n);
% z = logspace(az,bz,n);
%
% % Initialisation and loop to connstruct the Q matrix 
% Q = [];
% % Q = [91.5; 0.0924; 248;0;0;0];
% for i=1:n
%     for j=1:n
%         X = [0.01; 0.0464; y(i); 0.01; 27.8256; z(j)];
%         Q = [Q X];
%     end
% end

% Searching in the linear subspace close to the values found from the
% previous loop
% n = 100;
% ay = 200;
% by = 300;
% az = 0;
% bz = 10;
% y = linspace(ay,by,n);
% z = linspace(az,bz,n);
%
% % Initialisation and loop to connstruct the Q matrix 
% Q = [];
% % Q = [91.5; 0.0924; 248;0;0;0];
% for i=1:n
%     for j=1:n
%         X = [0.01; 0.0464; y(i); 0.01; 27.8256; z(j)];
%         Q = [Q X];
%     end
% end


% Tuning the dynamics in xy directions
% Searching in the log space 
tic
n = 10;
ay = -2;
by = -1;
az = -2;
bz = -1;
am = -2;
bm = -1;
ao = 1;
bo = 2;

y = logspace(ay,by,n);
z = logspace(az,bz,n);
m = logspace(am,bm,n);
o = logspace(ao,bo,n);

% % Initialisation and loop to connstruct the Q matrix 
Q=[];
for i=1:n
    for j=1:n
        for k=1:n
            for l=1:n
                X = [y(i); z(j); 223.2323; m(k); o(l); 0.3030];
                Q = [Q X];
            end
        end
    end
end

% Tuning the dynamics in xy directions
% Searching in the linear subspace close to the values found from the
% previous loop
% tic
% n = 10;
% ay = 0.08;
% by = 0.019;
% az = 0.03;
% bz = 0.07;
% am = 0.008;
% bm = 0.019;
% ao = 10;
% bo = 80;
% 
% y = linspace(ay,by,n);
% z = linspace(az,bz,n);
% m = linspace(am,bm,n);
% o = linspace(ao,bo,n);
% 
% % Initialisation and loop to connstruct the Q matrix 
% Q=[];
% for i=1:n
%     for j=1:n
%         for k=1:n
%             for l=1:n
%                 X = [y(i); z(j); 223.2323; m(k); o(l); 0.3030];
%                 Q = [Q X];
%             end
%         end
%     end
% end

[tuning_struct, i_opt] = lqr_tuning(x0,Q,params)
q = Q(:,i_opt)
tuning_struct(i_opt).InputCost
toc

% Output
filename = 'lqr_tuning_script.mat';
save(filename, "tuning_struct", "q")
