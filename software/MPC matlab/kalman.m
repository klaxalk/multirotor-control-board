function [ states, covariance ] = kalman( states, covariance, measurement, u, A, B, R, Q, C)
    
%% Krok predikce

states = A*states + B*u;

covariance = A*covariance*A' + R;

%% Krok korekce 

K = covariance*C'*((C*covariance*C' + Q)^-1);

states = states + K*(measurement - C*states);
covariance = (eye(5) - K*C)*covariance;

end