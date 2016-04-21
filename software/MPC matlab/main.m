clear all;

%% System matrices

% the system (and simulation) sampling time
dt = 0.0101;

% choose the uav model
uav = 1;
% 1 = 250tka, KK2
% 2 = Mikrokopter, KK2, ss = 15
% 3 = Tricopter, KK2 

%% 250 (prase), KK2 
if uav == 1
    A = [1 dt   0  0            0;
         0 1    dt 0            0;
         0 0    0  1            1;
         0 0    0  0.9777       0;
         0 0    0  0            1];

    B = [0; 0; 0; 4.4664e-05; 0];
end

%% Mikrokopter, KK2, stick scaling = 15
if uav == 2
    A = [1 dt   0  0            0;
         0 1    dt 0            0;
         0 0    0  1            1;
         0 0    0  0.9783       0;
         0 0    0  0            1];

    B = [0; 0; 0; 6.3151e-05; 0];
end

%% Tricopter, KK2 
if uav == 3
    A = [1 dt   0  0            0;
         0 1    dt 0            0;
         0 0    0  1            1;
         0 0    0  0.9759       0;
         0 0    0  0            1];

    B = [0; 0; 0; 4.4279e-05; 0];    
end

%%  aux variables

measurement_covariance = diag([0.0058, 0.19, 0, 0, 0]);

% number of system states
n_states = size(A, 1);

% number of inputs
u_size = size(B, 2);

% prediction horizon length
horizon_len = 200;

%% U matrix

n_variables = 20;

U = zeros(horizon_len, n_variables);

U(1:10, 1:10) = eye(10);
U(11:20, 11) = 1;
U(21:30, 12) = 1;
U(31:40, 13) = 1;
U(41:50, 14) = 1;
U(51:60, 15) = 1;
U(61:70, 16) = 1;
U(71:80, 17) = 1;
U(81:90, 18) = 1;
U(91:100, 19) = 1;
U(101:200, 20) = 1;
 
%% A_roof matrix
% n = prediction horizon length
% A_roof = [A;
%           A^2;
%           A^3;
%           ...;
%           A^n]; 

A_roof = zeros(horizon_len * n_states, n_states);

for i=1:horizon_len
   A_roof(((i-1)*n_states+1):(n_states*i), :) = A^i; 
end

%% B_roof matrix
% n = prediction horizon length
% B_roof = [B,        0,        0,   0;
%           AB,       B,        0,   0;
%           A^2B,     AB,       B,   0;
%           ...;
%           A^(n-2)B, A^(n-1)B, ..., 0;
%           A^(n-1)B, A^(n-2)B, ..., B;

B_roof = zeros(horizon_len * n_states, horizon_len);

for i=1:horizon_len
    for j=1:i
        B_roof((i-1)*n_states+1:i*n_states, ((j-1)*u_size+1):j*u_size) = (A^(i - j))*B;
    end
end

B_roof = B_roof*U;

%% Q_roof matrix
% n = number of system states
% Q = n*n
%     diagonal, penalizing control errors
% S = n*n
%     penalizing last error
% Q_roof = [Q,   0,   ...,  0;
%           0,   Q,   ...,  0;
%           ..., ..., Q,    0;
%           0,   ..., ...,  S];

% 250 (prase), KK2
if uav == 1
   
    Q = [1.07, 0, 0, 0, 0;
     0, 0, 0, 0, 0;
     0, 0, 0, 0, 0;
     0, 0, 0, 0, 0;
     0, 0, 0, 0, 0];
    
end

% mikrokopter, KK2
if uav == 2
    
    Q = [1.07, 0, 0, 0, 0;
         0, 0, 0, 0, 0;
         0, 0, 0, 0, 0;
         0, 0, 0, 0, 0;
         0, 0, 0, 0, 0];
     
end

% tricopter
if uav == 3
    
    Q = [1.07, 0, 0, 0, 0;
         0, 0, 0, 0, 0;
         0, 0, 0, 0, 0;
         0, 0, 0, 0, 0;
         0, 0, 0, 0, 0];
     
end

S = [10, 0, 0, 0, 0;
     0, 0, 0, 0, 0;
     0, 0, 0, 0, 0;
     0, 0, 0, 0, 0;
     0, 0, 0, 0, 0];
 
Q_roof = zeros(n_states * horizon_len, n_states * horizon_len);

for i=1:horizon_len
    Q_roof((i-1)*n_states+1:i*n_states, (i-1)*n_states+1:i*n_states) = Q; 
end

Q_roof(end-n_states+1:end, end-n_states+1:end) = S;

%% P_roof matrix
% penalizing control actions
% P_roof = [P,   0,   ...,  0;
%           0,   P,   ...,  0;
%           ..., ..., P,    0;
%           0,   ..., ...,  P];

P = 0.0000025;

P_roof = zeros(n_variables, n_variables);

for i=1:n_variables
    P_roof(i, i) = P;
end

for i=11:19
    P_roof(i, i) = 10*P; 
end

P_roof(20, 20) = 100*P;

%% H matrix
% the main matrix of the quadratic form

H = B_roof'*Q_roof*B_roof + P_roof;
H_inv = (0.5*H)^(-1);

%% the control loop

% simulation length
simu_len = 2500;

time(1) = 0;

% set the reference velocities

reference_vel = 0.25*cos(0:0.01:2*pi*20);
reference_pos = [zeros(1000, 1); integrate(reference_vel, dt)];

% reference position
x_ref = reference_pos;

% initial conditions
x(:, 1) = [3; 0; 0; 0; 0];

% prepare the vector of actions
u = zeros(horizon_len, u_size);

% history of used actions
u_hist(1) = 0;

% saturation akcnich zasahu
saturation = 1200;

% max speed for input preshaper
max_speed = 2.0;

% initialize the Kalmans covariance
kalmanCovariance = eye(5);

% initialize the estimated states
estimate(:, 1) = x(:, 1);

% initialize the pure integration 
xd_pure_integration(1) = x(1, 1);

% initialize the saturated input
u_sat = 0;

%% support matrices

% process noise
R_kalman = diag([1, 1, 1, 1, 0.02]);

% measurement noise
Q_kalman = diag([120]);
    
% measurement distribution
C_kalman = [0, 1, 0, 0, 0];

% do the simulation
for i=2:simu_len
    
    % integrate time     
    time(i) = time(i-1)+dt;
    
    % create the sensor measurement     
    measurement(:, i-1) = createMeasurement(x(:, i-1), measurement_covariance);
    
    % pure integration of px4flow velocity     
    xd_pure_integration(i) = xd_pure_integration(i-1) + measurement(2, i-1)*dt;

    % rune the kalman step     
    [estimate(:, i), kalmanCovariance] = kalman(estimate(:, i-1), kalmanCovariance, measurement(2, i-1), u_sat, A, B, R_kalman, Q_kalman, C_kalman); 
    
    % prepare the reference by input preshaper     
    reference = estimate(1, i);
    for j=2:horizon_len
        diference = reference(j-1) - x_ref(j+i-1);
                 
        if (diference > max_speed*dt)
            diference = max_speed*dt;
        elseif (diference < -max_speed*dt)
            diference = -max_speed*dt;
        end
        
        reference(j) = reference(j-1) - diference;
    end

    % prepare the reference (create the vector of all states from the position reference)     
    my_ref = zeros(n_states*horizon_len, 1);
    my_ref(1:n_states:n_states*horizon_len, 1) = reference;
    
    % prepare the lineare part of the qudratic function     
    X_0 = A_roof*(estimate(:, i)) - my_ref;
    c = (X_0'*Q_roof*B_roof)';
    
    % find the global optimum     
    u_cf = H_inv*(c./(-2));
        
    % stretch the action vector to the whole horizon     
    u_cf = U*u_cf;
    
    % predict the system using the whole control horizon
    x_cf(:, 1) = estimate(:, i);
    for j=2:horizon_len
        x_cf(:, j) = A*x_cf(:, j-1) + B*u_cf(j-1);
    end

    % take the first action
    u_sat = u_cf(1);
    
    % saturate the control action
    if (u_sat > saturation)
        u_sat = saturation;
    elseif (u_sat < -saturation)
        u_sat = -saturation;
    end
    
    u_hist(i) = u_sat;

    % Compute new states     
    x(:, i) = A*x(:, i-1) + (B*u_sat);
    
    % plot during the simulation
    if (mod(i, 3) == 0)
    figure(2);
    subplot(1, 2, 1);
    hold off
    plot(linspace(0, dt*(i+horizon_len), i+horizon_len), x_ref(1:(i+horizon_len)), 'r');
    hold on
    plot(linspace(0, dt*i, i), estimate(1, 1:i), 'b');
    plot(linspace(dt*i, dt*(i+horizon_len), horizon_len), x_cf(1, 1:horizon_len), 'g--');
    subplot(1, 2, 2);
    plot(u_cf);
    drawnow;
    end
        
end

%% Plot the final results 

figure(1);

% plot the reference and estimate

subplot(2, 2, 1);
hold off
plot(linspace(0, dt*simu_len, simu_len), x_ref(1:simu_len), 'r');
hold on
plot(time, estimate(1, 1:i), 'b');
plot(linspace(i*dt, dt*(i+horizon_len), horizon_len), x_cf(1, :),'--r');
title('Position estimate');
ylabel('Position [m]');
xlabel('Time [s]');
legend('Reference', 'KF Estimate');

% plot the control action

subplot(2, 2, 2);
hold off
plot(time, u_hist, 'r');
hold on
plot(time, 0*(1:simu_len), 'b');
title('Control action');
ylabel('Control acton []');
xlabel('Time [s]');
axis([0, dt*simu_len, -saturation, saturation]);
legend('Control action');

% plot the position estimates

subplot(2, 2, 3);
hold off
plot(time(1:end-1), estimate(1, 1:(i-1)), 'r');
hold on
plot(time, x(1, 1:i), 'b');
plot(time, xd_pure_integration, 'g');
xlabel('Time [s]');
ylabel('Position [m]');
legend('KF estimate', 'Simulation ground truth', 'Pure integration');
title('Position estimates');

% plot the velocities

subplot(2, 2, 4);
hold off
plot(time(1:end-1), measurement(2, 1:(i-1)), 'r');
hold on
plot(time(1:end-1), estimate(2, 1:(i-1)), 'b');
xlabel('Time [s]');
ylabel('Speed [m/s]');
legend('Measurement', 'KF Estimate');
title('Velocities');