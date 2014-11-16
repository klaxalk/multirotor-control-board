function [ states ] = createMeasurement( states, covariance )

    n_states = size(states, 1);

    states = states + covariance*randn(n_states, 1);

end

