function visualize_results(x_history, u_history, x_min, x_max, u_min, u_max)

%
% Inputs:
%   x_history - State history
%   u_history - Input history
%   x_min     - Lower bound for state constraints
%   x_max     - Upper bound for state constraints
%   u_min     - Lower bound for input constraints
%   u_max     - Upper bound for input constraints

    % Time vector
    T = size(x_history, 1) - 1;
    t = 0:T;
    t_u = 0:T-1;
    
    figure('Name', 'CSB-MPC Simulation Results: States', 'NumberTitle', 'off');
    

    subplot(2, 1, 1);
    plot(t, x_history(:, 1), 'b-', 'LineWidth', 2);
    grid on;
    xlabel('Time step');
    ylabel('State x_1');
    title('State x_1 Trajectory (Unconstrained)');
    legend('x_1');
    
    subplot(2, 1, 2);
    plot(t, x_history(:, 2), 'b-', 'LineWidth', 2);
    hold on;
    plot(t, x_max(2)*ones(size(t)), 'r--');
    plot(t, x_min(2)*ones(size(t)), 'r--');
    grid on;
    xlabel('Time step');
    ylabel('State x_2');
    title('State x_2 Trajectory (Constrained: |x_2| \leq 1.0)');
    legend('x_2', 'Constraints');
    

    figure('Name', 'CSB-MPC Simulation Results: Inputs', 'NumberTitle', 'off');
    

    plot(t_u, u_history, 'g-', 'LineWidth', 2);
    hold on;
    plot(t_u, u_max*ones(size(t_u)), 'r--');
    plot(t_u, u_min*ones(size(t_u)), 'r--');
    grid on;
    xlabel('Time step');
    ylabel('Control input u');
    title('Control Input Trajectory');
    legend('u', 'Constraints');
    

    figure('Name', 'CSB-MPC Simulation Results: State Space', 'NumberTitle', 'off');

    plot(x_history(:, 1), x_history(:, 2), 'b-', 'LineWidth', 2);
    hold on;
    plot(x_history(1, 1), x_history(1, 2), 'go', 'MarkerSize', 10, 'LineWidth', 2); % Start point
    plot(x_history(end, 1), x_history(end, 2), 'ro', 'MarkerSize', 10, 'LineWidth', 2); % End point

    x1_range = [min(x_history(:, 1))-1, max(x_history(:, 1))+1];
    plot(x1_range, [x_max(2), x_max(2)], 'r--');
    plot(x1_range, [x_min(2), x_min(2)], 'r--');
    
    grid on;
    xlabel('State x_1');
    ylabel('State x_2');
    title('State Space Trajectory (Only x_2 Constrained)');
    legend('Trajectory', 'Initial State', 'Final State', 'Constraints');
    

    x2_violations = sum(x_history(:, 2) > x_max(2) | x_history(:, 2) < x_min(2));
    u_violations = sum(u_history > u_max | u_history < u_min);
    
    fprintf('Constraint violations summary (Example 1):\n');
    fprintf('  State x2: %d violations (%.2f%%)\n', x2_violations, 100*x2_violations/length(t));
    fprintf('  Input u: %d violations (%.2f%%)\n', u_violations, 100*u_violations/length(t_u));
end 