function [ states ] = createMeasurement( states, covariance )

    n_states = size(states, 1);

    states = states + covariance*randn(n_states, 1);
    
    if (states(2) > 0.5)
       states(2) = 0.5;
    elseif (states(2) < -0.5)
        states(2) = -0.5;
    end

end

