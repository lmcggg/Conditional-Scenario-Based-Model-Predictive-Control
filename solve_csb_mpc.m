function [v_opt, x_pred, u_pred] = solve_csb_mpc(x0, N, C, conditional_scenarios, ...
                                             probabilities, A_nominal, B_nominal, G, K, ...
                                             Q, R, P, x_min, x_max, u_min, u_max, example_num)
% SOLVE_CSB_MPC Solve the CSB-MPC optimization problem
%
% Inputs:
%   x0                    - Initial state
%   N                     - Prediction horizon
%   C                     - Number of conditional scenarios
%   conditional_scenarios - Conditional scenarios (N x C x n_delta)
%   probabilities         - Probabilities of scenarios (N x C)
%   A_nominal, B_nominal  - Nominal system matrices
%   G                     - Disturbance matrix
%   K                     - Feedback gain matrix
%   Q, R, P               - Cost matrices
%   x_min, x_max          - State constraints
%   u_min, u_max          - Input constraints
%   example_num           - Example number (1 or 2) to determine simulation mode
%
% Outputs:
%   v_opt  - Optimal control increments
%   x_pred - Predicted states
%   u_pred - Predicted inputs

    % Get dimensions
    n_x = length(x0);
    n_u = size(B_nominal, 2);
    n_delta = size(conditional_scenarios, 3);
 
    if nargin < 17
        example_num = 1;
    end
    

    x_pred = zeros(C, N+1, n_x);
    u_pred = zeros(C, N, n_u);
    

    cvx_clear;

    cvx_begin quiet

        variable v(N, n_u)  
        

        expression total_cost
        total_cost = 0;
        

        for j_scenario = 1:C

            x_0j = x0; 
            
            cost_scenario_j = 0;
            

            x_traj = cell(N+1, 1);
            u_traj = cell(N, 1);
            
            x_traj{1} = x_0j;
            

            for k_cvx = 1:N

                delta_current = squeeze(conditional_scenarios(k_cvx, j_scenario, :));
                

                A = A_nominal;
                B = B_nominal;
                

                x_current_pred = x_traj{k_cvx};
                

                u_current = K * x_current_pred + v(k_cvx, :)';
                u_traj{k_cvx} = u_current;
                u_current_pred = u_current;  
                

                x_next = A * x_current_pred + B * u_current_pred + G * delta_current;
                x_traj{k_cvx+1} = x_next;
                
 
                if k_cvx == 1
                    weight_state = 1/C;
                else                   
                    weight_state = probabilities(k_cvx-1, j_scenario);
                end
                cost_scenario_j = cost_scenario_j + weight_state * quad_form(x_current_pred, Q);
                
  
                weight_input = probabilities(k_cvx, j_scenario);
                cost_scenario_j = cost_scenario_j + weight_input * quad_form(u_current_pred, R);

                if example_num == 1
         
                    x_next(2) <= x_max(2); 
                    x_next(2) >= x_min(2);  
                else
           
                    x_next <= x_max;
                    x_next >= x_min;
                end
                
                u_current <= u_max;
                u_current >= u_min;
            end
            
         
            x_terminal_pred = x_traj{N+1};
            weight_terminal = probabilities(N, j_scenario);
            cost_scenario_j = cost_scenario_j + weight_terminal * quad_form(x_terminal_pred, P);
            
         
            total_cost = total_cost + cost_scenario_j;
        end
   
        minimize(total_cost / C)
        
    cvx_end

    if strcmp(cvx_status, 'Solved') || strcmp(cvx_status, 'Inaccurate/Solved')
        v_opt = v;

        for j = 1:C
          
            x_pred(j, 1, :) = x0;
            
            for k = 1:N
             
                x_k = squeeze(x_pred(j, k, :));
                
              
                u_k = K * x_k + v_opt(k, :)';
                u_pred(j, k, :) = u_k;
                
                
                delta_current = squeeze(conditional_scenarios(k, j, :));
                
       
                A = A_nominal;
                B = B_nominal;
                
    
                x_next = A * x_k + B * u_k + G * delta_current;
                x_pred(j, k+1, :) = x_next;
            end
        end
        
        fprintf('CSB-MPC optimization (Example %d) solved successfully.\n', example_num);
    else
        warning('CSB-MPC optimization (Example %d) failed with status: %s', example_num, cvx_status);
        v_opt = zeros(N, n_u);  
    end
end 