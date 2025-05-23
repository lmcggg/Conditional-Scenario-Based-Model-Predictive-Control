# Conditional Scenario-Based Model Predictive Control (CSB-MPC)

This repository contains a MATLAB implementation of the Conditional Scenario-Based Model Predictive Control (CSB-MPC) algorithm. The implementation is based on the mathematical derivation provided in the paper "Conditional scenario-based model predictive control".

## Overview

CSB-MPC is an advanced control technique that handles systems with uncertainties by using scenario-based optimization. The key innovation is the use of conditional scenarios to reduce computational complexity while maintaining robustness.

## Files Description

### Main Scripts
- `csb_mpc_receding_horizon.m`: Full receding horizon control implementation for Example 1 from the paper

### Supporting Functions
- `generate_truncated_mvn.m`: Function to generate samples from a truncated multivariate normal distribution
- `reduce_scenarios.m`: Implementation of the conditional scenario reduction algorithm
- `solve_csb_mpc.m`: Function to solve the CSB-MPC optimization problem
- `visualize_results.m`: Function to visualize the simulation results

## Requirements

- MATLAB (R2018b or newer recommended)
- Optimization toolbox (for solving the CSB-MPC optimization problem)
- Statistics and Machine Learning Toolbox (for `mvnrnd` function)

## Implementation Details

### Example 1 (Pure Additive Disturbance)

The current implementation focuses on Example 1 from the paper, which features:
- Second-order discrete system with pure additive disturbance
- Constraint only on the second state component: |x2| ≤ 1.0
- No parametric uncertainty in system matrices
- Disturbance follows truncated multivariate normal distribution

### System Model

The implementation considers a system model with pure additive disturbance:
```
x_{i+1} = A_nominal*x_i + B_nominal*u_i + G*w(δ_i)
```

where δ_i is a random vector following a truncated multivariate normal distribution.

### Main Steps of the Algorithm

1. **Generate Main Scenario Set**: Generate S scenarios from the truncated multivariate normal distribution
2. **Conditional Scenario Reduction**: Reduce the number of scenarios using the conditional scenario approach
3. **Solve Optimization Problem**: Formulate and solve the CSB-MPC optimization problem
4. **Apply Control**: Apply the first control action and simulate the system response

### Key Parameters

- State dimension (n_x): 2
- Input dimension (n_u): 1
- Random vector dimension (n_delta): 2
- Number of main scenarios (S): 3000
- Prediction horizon (N): 15
- Number of subintervals for each random variable (E): 7
- Number of conditional scenarios (C): n_delta * E = 14

### Performance Metrics

The implementation provides several performance metrics:
- Computational efficiency (execution time per step)
- Constraint violation frequency
- Control performance (state and input costs)
- State trajectory visualizations
- Control input profiles

## Usage

To run the CSB-MPC implementation for Example 1:
```matlab
csb_mpc_receding_horizon
```

This will execute the full receding horizon control simulation and display the results, including:
- State and input trajectories
- Computation time per step
- Control increments
- Performance analysis (constraint violations and cost metrics)

## References

Lucia, S., Subramanian, S., & Engell, S. (2022). Conditional scenario-based model predictive control. International Journal of Robust and Nonlinear Control, 32(11), 6410-6428. 