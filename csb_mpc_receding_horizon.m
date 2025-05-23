%% Parameters Initialization
clear; clc; close all;

% System dimensions
n_x = 2;             
n_u = 1;             
n_delta = 2;         

% Scenario parameters
S = 5000;            %  of main scenarios
N = 15;             
E = 7;               %  subintervals for each random variable
C = n_delta * E;     %  conditional scenarios

% Distribution parameters for random vector delta (Example 1 in paper)
mu = [0, 0];         % Mean of the multivariate normal distribution
Sigma = [0.0004, 0.0017; 0.0017, 0.01]; % Covariance matrix from Example 1
bounds = [-0.05, 0.05; -0.2, 0.2];      % Truncation bounds for each dimension

% System matrices (Example 1 in paper)
A_nominal = [1, 0.93; 0, 1];   % Nominal A matrix
B_nominal = [0.28; 0.82];      % Nominal B matrix
G = eye(2);                    % Disturbance matrix (identity as per Example 1)
K = [-0.7421, -1.5891];        % Feedback gain matrix from Example 1

% Cost matrices (Example 1 in paper)
Q = diag([1, 1]);              % State weight
R = 0.1;                       % Input weight
P = [2.3026, 0.8572; 0.8572, 1.6983]; % Terminal weight from Example 1

% Initial state (Example 1 in paper)
x0 = [8; 0.7];

% Constraints (Example 1 in paper)
% Only x2 has constraint |x2| <= 1.0
% For x1, we use very large bounds to effectively remove the constraint
x_min = [-1e6; -1.0];  % Only constrain x2
x_max = [1e6; 1.0];    % Only constrain x2
u_min = -0.8;          % Input constraint
u_max = 0.8;           % Input constraint

% Simulation parameters
T_sim = 20;          % Simulation time (as in Example 1)

%% Receding Horizon Control Loop
fprintf('Starting CSB-MPC receding horizon control simulation...\n');

% Initialize storage
x_history = zeros(T_sim+1, n_x);
u_history = zeros(T_sim, n_u);
v_history = zeros(T_sim, n_u);
comp_time = zeros(T_sim, 1);

% Set initial state
x_history(1, :) = x0';
x_k = x0;

% Main control loop
for k = 1:T_sim
    fprintf('Step %d/%d: ', k, T_sim);
    
    % Start timing for computation
    tic;
  
    delta_scenarios = generate_truncated_mvn(S*N, mu, Sigma, bounds);
    delta_scenarios = reshape(delta_scenarios, [N, S, n_delta]);
  
    [conditional_scenarios, probabilities] = reduce_scenarios(delta_scenarios, n_delta, E);

    [v_opt, x_pred, u_pred] = solve_csb_mpc(x_k, N, C, conditional_scenarios, probabilities, ...
                                           A_nominal, B_nominal, G, K, Q, R, P, ...
                                           x_min, x_max, u_min, u_max);
    

    v_k = v_opt(1, :)';  
    v_history(k, :) = v_k';
    

    u_k = K * x_k + v_k;
    u_history(k, :) = u_k';
    

    delta_k = generate_truncated_mvn(1, mu, Sigma, bounds);
    
 
    A = A_nominal;
    B = B_nominal;
    
   
    x_next = A * x_k + B * u_k + G * delta_k';
    x_history(k+1, :) = x_next';
    
    x_k = x_next;

    comp_time(k) = toc;
    
    fprintf('State = [%.2f, %.2f], Input = %.2f, Comp. Time = %.3f s\n', ...
            x_k(1), x_k(2), u_k, comp_time(k));
end

%% Visualize Results
fprintf('\nVisualization of receding horizon control results...\n');
visualize_results(x_history, u_history, x_min, x_max, u_min, u_max);
figure('Name', 'CSB-MPC Computation Time', 'NumberTitle', 'off');
plot(1:T_sim, comp_time, 'b-o', 'LineWidth', 1.5);
grid on;
xlabel('Time step');
ylabel('Computation time (s)');
title('CSB-MPC Computation Time per Step');
figure('Name', 'CSB-MPC Control Increments', 'NumberTitle', 'off');
plot(1:T_sim, v_history, 'r-o', 'LineWidth', 1.5);
grid on;
xlabel('Time step');
ylabel('Control increment v');
title('CSB-MPC Control Increments');

%% Performance Analysis
fprintf('\nPerformance Analysis:\n');
avg_comp_time = mean(comp_time);
fprintf('  Average computation time: %.3f seconds\n', avg_comp_time);

x2_violations = sum(x_history(:, 2) > x_max(2) | x_history(:, 2) < x_min(2));
u_violations = sum(u_history > u_max | u_history < u_min);

fprintf('  State x2 violations: %d (%.2f%%)\n', x2_violations, 100*x2_violations/(T_sim+1));
fprintf('  Input u violations: %d (%.2f%%)\n', u_violations, 100*u_violations/T_sim);

J_state = sum(sum(x_history(1:end-1, :).^2 * Q));
J_input = sum(u_history.^2 * R);
J_total = J_state + J_input;

fprintf('  Cost metrics:\n');
fprintf('    State cost: %.2f\n', J_state);
fprintf('    Input cost: %.2f\n', J_input);
fprintf('    Total cost: %.2f\n', J_total);

fprintf('\nCSB-MPC receding horizon control simulation (Example 1) completed.\n'); 