function samples = generate_truncated_mvn(n, mu, Sigma, bounds)
%
% Inputs:
%   n       - Number of samples to generate
%   mu      - Mean vector (1 x d)
%   Sigma   - Covariance matrix (d x d)
%   bounds  - Bounds for truncation (d x 2), each row contains [lower, upper] bounds
%
% Output:
%   samples - Matrix of samples (n x d)

    % Get dimension of random vector
    d = length(mu);
    
    % Check inputs
    if size(bounds, 1) ~= d || size(bounds, 2) ~= 2
        error('Bounds should be a d x 2 matrix');
    end
    
    % Initialize output
    samples = zeros(n, d);
    
   
    oversampling_factor = 5;
    candidate_samples = mvnrnd(mu, Sigma, n * oversampling_factor);
    
    % Apply truncation
    valid_samples = true(size(candidate_samples, 1), 1);
    for i = 1:d
        valid_samples = valid_samples & ...
                       (candidate_samples(:, i) >= bounds(i, 1)) & ...
                       (candidate_samples(:, i) <= bounds(i, 2));
    end
    
    % Extract valid samples
    valid_candidate_samples = candidate_samples(valid_samples, :);
    
    if size(valid_candidate_samples, 1) < n
        warning('Not enough valid samples after truncation. Generating more...');
      
        additional_samples = generate_truncated_mvn(n - size(valid_candidate_samples, 1), ...
                                                  mu, Sigma, bounds);
        samples = [valid_candidate_samples; additional_samples];
    else
     
        samples = valid_candidate_samples(1:n, :);
    end
end 