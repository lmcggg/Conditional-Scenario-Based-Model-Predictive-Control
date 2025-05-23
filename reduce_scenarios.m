function [conditional_scenarios, probabilities] = reduce_scenarios(delta_scenarios, n_delta, E)
% REDUCE_SCENARIOS Implement conditional scenario reduction algorithm based on
%
% Inputs:
%   delta_scenarios - Main scenario set (N x S x n_delta)
%   n_delta        - Dimension of random vector
%   E              - Number of subintervals for each random variable
%
% Outputs:
%   conditional_scenarios - Reduced conditional scenarios (N x C x n_delta)
%   probabilities        - Probabilities of conditional scenarios (N x C)

   
    [N, S, ~] = size(delta_scenarios);
    C = n_delta * E;  
    
    % Initialize temporary storage for Algorithm 1 outputs at each time step
    temp_scenarios = zeros(N, C, n_delta);  
    temp_probs = zeros(N, C);               
    
    % Step 1: Apply Algorithm 1 at each time step i
    for i = 1:N
        % Extract scenarios for current time step
        current_deltas = squeeze(delta_scenarios(i, :, :)); % S x n_delta
        
        % Initialize counter for conditional scenarios
        scenario_idx = 1;
        
        % Apply Algorithm 1 to reduce scenarios for this time step
        for n = 1:n_delta  % For each random variable ξ_n
            % Find min and max for current random variable
            min_xi = min(current_deltas(:, n));
            max_xi = max(current_deltas(:, n));
            
            % Create E subintervals for this random variable
            interval_width = (max_xi - min_xi) / E;
            edges = min_xi + (0:E) * interval_width;
           
            for e = 1:E
                
                if e < E
                    idx = (current_deltas(:, n) >= edges(e)) & (current_deltas(:, n) < edges(e+1));
                else
                 
                    idx = (current_deltas(:, n) >= edges(e)) & (current_deltas(:, n) <= edges(e+1));
                end

                subset = current_deltas(idx, :);
                
      
                if ~isempty(subset)
                   
                    cond_scenario = mean(subset, 1);

                    prob = size(subset, 1) / S;
                else
                
                    cond_scenario = zeros(1, n_delta);
                    prob = 0;
          
                end
     
                temp_scenarios(i, scenario_idx, :) = cond_scenario;
                temp_probs(i, scenario_idx) = prob;
                
                scenario_idx = scenario_idx + 1;
            end
        end

        if sum(temp_probs(i, :)) > 0
            temp_probs(i, :) = temp_probs(i, :) / sum(temp_probs(i, :));
        else

            temp_probs(i, :) = ones(1, C) / C;
        end
    end
    
    % Step 2: Apply Algorithm 2 to construct C conditional scenario sequences with better diversity

    conditional_scenarios = zeros(N, C, n_delta);
    probabilities = zeros(N, C);
    

    for j_seq = 1:C
        for k_pred_step = 1:N

            m_selected = mod(j_seq + k_pred_step - 2, C) + 1;

            conditional_scenarios(k_pred_step, j_seq, :) = temp_scenarios(k_pred_step, m_selected, :);
            probabilities(k_pred_step, j_seq) = temp_probs(k_pred_step, m_selected);
        end
    end
    

    % Uncomment this section to use random permutations instead of the deterministic mapping
    
    % % Create random permutations for each prediction step
    % rng(42); % Set random seed for reproducibility
    % for k_pred_step = 1:N
    %     % Generate a random permutation for this time step
    %     perm_indices = randperm(C);
    %     
    %     % Apply the permutation to rearrange scenarios at this time step
    %     for j_seq = 1:C
    %         m_selected = perm_indices(j_seq);
    %         conditional_scenarios(k_pred_step, j_seq, :) = temp_scenarios(k_pred_step, m_selected, :);
    %         probabilities(k_pred_step, j_seq) = temp_probs(k_pred_step, m_selected);
    %     end
    % end
    
    % Implementation option 3: Hybrid approach with structured randomness
    % Uncomment this section to use a hybrid approach for better diversity
    
    % % Set random seed for reproducibility
    % rng(42);
    % 
    % % Pre-generate structured index mappings for each sequence
    % % This ensures each sequence has a unique but diverse sampling pattern
    % index_mappings = zeros(C, N);
    % for j_seq = 1:C
    %     % Create a base permutation for this sequence
    %     base_perm = randperm(C);
    %     
    %     % Add structured variation based on time step
    %     for k_pred_step = 1:N
    %         % Rotate the permutation by different amounts for each time step
    %         shift = mod(k_pred_step * floor(sqrt(j_seq)), C);
    %         index_mappings(j_seq, k_pred_step) = base_perm(mod(shift, C) + 1);
    %     end
    % end
    % 
    % % Apply the structured index mappings
    % for j_seq = 1:C
    %     for k_pred_step = 1:N
    %         m_selected = index_mappings(j_seq, k_pred_step);
    %         conditional_scenarios(k_pred_step, j_seq, :) = temp_scenarios(k_pred_step, m_selected, :);
    %         probabilities(k_pred_step, j_seq) = temp_probs(k_pred_step, m_selected);
    %     end
    % end
end 