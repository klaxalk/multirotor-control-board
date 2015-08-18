states = [0; 0; 0; 0; 0];
covariance = eye(5);

from = 1001;
to = 4000;
dataLen = to-from;

data = load('LOG00029.TXT');

data = data(from:to, :);
data(:, 1) = filtr_extremnich_hodnot(data(:, 1));

dt = 0.0114;

g = 9.8;

A = [1 dt 0 0 0;
     0 1 dt 0 0;
     0 0 0 1 1;
     0 0 0 0.9712 0;
     0 0 0 0 1];
 
B = [0;
     0;
     0;
     0.0004;
     0];

% process noise
R = diag([1, 1, 1, 1, 1]);

% measurement noise
Q = 10;

% measurement distribution
C = [1 0 0 0 0];

states(1, 1) = data(1, 1);
states(2, 1) = 0;
states(3, 1) = 0
states(5, 1) = -9;

for i=1:dataLen
    [states(:, i+1) covariance] = kalman(states(:, i), covariance, data(i, 1), data(i, 2), A, B, R, Q, C);
end

figure(12);
% subplot(2, 2, 1);
hold off;
plot(data(:, 1));
hold on;
plot(states(1, :), 'g');

% 
% subplot(2, 2, 2);
% plot(states(2, :), 'g');

figure(13);
plot(states(3, :), 'g');

