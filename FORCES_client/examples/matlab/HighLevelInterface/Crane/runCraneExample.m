function runCraneExample()
% This example solves an optimization problem for a crane and shows how one 
% can exploit linear subsystems of the dynamical system in FORCESPRO for 
% improved performance. The crane is described by the following simple 
% continuous-time nonlinear model:
% 
%   dxC/dt = vC
%   dvC/dt = -vC/tau + (AC*uC)/tau
%   dxL/dt = vL
%   dvL/dt = -vL/tau + (AL*uL)/tau
%   dTheta/dt = omega
%   dOmega/dt = -((-vC/tau + (AC*uC)/tau)*cos(theta) + 9.81*sin(theta) + 2*vL*omega)/xL 
%   duC/dt = uCR
%   duL/dt = uLR.
% 
% Here AC and AL denote the gains of suitable transfer functions for the 
% winch dynamics and tau denotes the time constant of the winch dynamics in
% seconds. 
%
% uCR and and uLR constitute the control variables and denote the voltage 
% rate for the horizontal and rotating actuator respectively. 
%
% The state variables consist of xC (cart position), vC (cart velocity), xL
% (cable length), vL (rate of change of cable length), theta (angle of
% pendulum), omega (rate of change of angle), uC (voltage for horizontal 
% actuator) and uL (voltage for rotating actuator).
%
% For further details on the crane model we refer to the following two
% papers:
% 
% Vukov, Milan & Van Loock, Wannes & Houska, Boris & Ferreau, Joachim & 
% Swevers, Jan & Diehl, Moritz. (2012). Experimental validation of nonlinear 
% MPC on an overhead crane using automatic code generation. Proceedings of 
% the American Control Conference. 6264-6269. 10.1109/ACC.2012.6315390. 
%
% Quirynen, Rien & Gros, Sebastien & Diehl, Moritz. (2013). Efficient NMPC 
% for nonlinear models with linear subsystems. Proceedings of the IEEE 
% Conference on Decision and Control. 5101-5106. 10.1109/CDC.2013.6760690. 
%
% The crane starts in a certain position and the optimization problem is to
% track reference values for the cart's position and the length of the
% cable. 
%
% There are bounds on the control inputs as well as their integrals.
% 
% This example models the task as a MPC problem using the PDIP_NLP method.
% 
% See also FORCES_NLP
% 
% (c) Embotech AG, Zurich, Switzerland, 2013-2021.
    
    clc;
    close all;

    %% Define crane model
    % Dimensions
    model.N     = 20;       % horizon length
    model.nvar  = 10;       % number of variables
    model.neq   = 8;        % number of equality constraints
    model.nh    = 0;        % number of inequality constraint functions
    model.npar  = 2;        % number of parameters
    nx = 8;
    nu = 2;

    % Dynamics
    model.E = [zeros(nx,nu), eye(nx)];
    model.continuous_dynamics = @(x,u,p) ode(x,u,p);

    % Bounds
    model.lb = [ -100, -100, -inf, -inf, -inf, -inf, -inf, -inf, -10, -10 ]; 
    model.ub = [ +100, +100, +inf, +inf, +inf, +inf, +inf, +inf, +10, +10 ];

    % Initial state
    xinitidx = 3:10;
    model.xinitidx = xinitidx;

    % Least squares objective function
    model.LSobjective = @(z, p) LScost(z, p);

    %% Set codeoptions to specify solver settings
    codeoptions = getOptions('CraneSolver');
    Ts = 1/100; % sampling time
    codeoptions.nlp.integrator.Ts = Ts;
    nodes = 4;
    codeoptions.nlp.integrator.nodes = nodes;
    codeoptions.nlp.integrator.type = 'ERK4';
    codeoptions.nlp.integrator.attempt_subsystem_exploitation = 1; % Enable subsystem exploitation for performance
    codeoptions.printlevel = 0;
    codeoptions.nlp.hessian_approximation = 'gauss-newton';
    codeoptions.server = 'https://forces.embotech.com/';

    % Generate solver
    stages = FORCES_NLP(model, codeoptions);

    %% Simulation
    totalTime = 100; % number of seconds
    nSamples = totalTime / Ts;

    t = 0;
    all_refs = zeros(2, nSamples);
    cart_data = zeros(2, nSamples);
    time = zeros(1, nSamples);
    x = [ 0.15; 0; 0.7; 0; 0; 0; 0; 0];
    for ii = 1:nSamples

        % get current reference
        ref = getRef(t, totalTime);

        % record data for plotting
        all_refs(:,ii) = ref;
        cart_data(:,ii) = x([1, 3]);
        time(ii) = t;    

        % set up problem data
        problem.xinit = x;
        problem.x0 = repmat([0;0;x],model.N,1);    
        problem.all_parameters = repmat(ref,model.N,1);

        % call FORCESPRO solver and check exit status
        [solution, exitflag, info] = CraneSolver(problem);    
        if exitflag ~= 1
            error('Encountered solver failure.');
        end

        % extract control and update state
        u = solution.x01(1:2);
        x = RK4( x, u, @(x,u,p) ode(x,u,p), Ts, ref, nodes);
        t = t + Ts;
    end

    %% Plot results
    figure('Name','Cart position vs time');clc;
    plot(time, cart_data(1,:), 'b', 'LineWidth', 2); hold on;
    plot(time, all_refs(1,:), 'r', 'LineWidth', 2); hold on;
    xlabel('Simulation time (s)'); ylabel('Cart position (m)');grid on;

    figure('Name','Cable length vs time');clc;
    plot(time, cart_data(2,:), 'b', 'LineWidth', 2); hold on;
    plot(time, all_refs(2,:), 'r', 'LineWidth', 2); hold on;
    xlabel('Simulation time (s)'); ylabel('Cable length (m)');grid on;
end

%% Utility functions

function [ r ] = getRef(t,totalTime)
    % Returns the reference for the current sampling time
    refs = {[0.2; 0.7], [0.4; 0.4], [0.2; 0.6], [0.3; 0.8]};
    nRefs = length(refs);
    breakTimes = totalTime / nRefs;

    r = refs{1};
    for ii = 1:nRefs
        if (t >= (ii-1) * breakTimes) && (t < ii * breakTimes)
            r = refs{ii};
            break
        end
    end

    end

    function [ r ] = LScost(z,p)
    ep = 1e-5;
    cst = 50;
    sep = sqrt(ep);
    scst = sqrt(cst);
    r = [ sep*z(1); sep*z(2); scst*(z(3)-p(1)); sep*z(4); scst*(z(5)-p(2)); sep*z(6); sep*z(7); sep*z(8); sep*z(9); sep*z(10)];
end

function dx = ode(x,u,p)
 
    g = 9.81; % gravitational constant
    AC = 0.0474; % gain of GC(s) in m/s/V
    AL = 0.0341; % gain of GL(s) m/s/V
    tau = 0.0247; % time constant of winch dynamics in seconds


    uCR = u(1); % voltage rate for horizontal actuator
    uLR = u(2); % voltage rate for rotating actuator

    xC = x(1); % cart position
    vC = x(2); % cart velocity
    xL = x(3); % cable length
    vL = x(4); % rate of change of cable length
    theta = x(5); % angle of pendulum
    omega = x(6); % rate of change of angle
    uC = x(7); % voltage for horizontal actuator
    uL = x(8); % voltage for rotating actuator

    aT = -(1/tau)*vC + (AC/tau)*uC;
    aL = -(1/tau)*vL + (AL/tau)*uL;

    dx = [  vC; ...
            aT; ...
            vL; ...
            aL; ...
            omega; ...
            -(1/xL)*(aT*cos(theta) + g*sin(theta) + 2*vL*omega); ...
            uCR; ...
            uLR   ];
end