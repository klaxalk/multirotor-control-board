states = [0; 0; 0];
covariance = eye(3);

measurement = 1;

u = -100;

dt = 0.0114;

A = [1 dt 0;
     0 1 dt;
     0 0 dt*82.6135];
 
B = [0; 0; 0.0105*dt];

for i=1:100
    [states covariance] = kalman(states, covariance, measurement, u, A, B);
end

states