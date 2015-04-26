clear all;

data = load('LOG00006.TXT');

%% data labels

dt = 1;
velocity = 2;
altitude = 3;
input = 4;

%% data margins

% okrojení dat
from = 500;
to = from+800;

% od-do fitování
plot_window = from:to;

% od-do identifikace
margin = 200;
iden_window = (from+margin):(to-margin); 

%% integrate dt to time vector

time_plot = integrate(data(plot_window, dt));
time_identifikace = integrate(data(iden_window, dt))+time_plot(margin);

%% position

% integruju rychlost
position = integrate(data(plot_window, velocity), data(plot_window, dt)); 

%% velocity

% fituji rychlost polynomem
velocity_pol = polyfit(time_plot', data(plot_window, velocity)', 50);
velocity_fit = polyval(velocity_pol, time_identifikace);

%% acceleration

% derive the velocity polynomial
acceleration_pol = polyder(velocity_pol);
acceleration_fit = polyval(acceleration_pol, time_identifikace);

%% input

% shift input of the offset
data(:, input) = data(:, input) - mean(data(plot_window(1:50), input))*0.960;

%% identify transfare input -> acceleration
input_window = data(iden_window, input);
dt_window = data(iden_window, dt);

B = acceleration_fit(2:end);
A = [acceleration_fit(1:end-1) input_window(1:end-1)];

P = A\B;
P0 = P(1)
P1 = P(2)

%% Estimating all values from input

acceleration_est = firstOrder(data(plot_window, input), P);

velocity_est = integrate(acceleration_est, data(plot_window, dt));

position_est = integrate(velocity_est, data(plot_window, dt));

%% plot position

hFig = figure(1);

subplot(2, 2, 1);
hold off
plot(time_plot, position, 'r--', 'LineWidth', 1.5);
hold on
plot(time_plot, position_est, 'b', 'LineWidth', 1.5);
title('Position');
legend('Integrated measurement', 'Estimated in open-loop');
xlabel('Time [s]');
ylabel('Position [m]');
axis([0 time_plot(end) -1.2 2.5]);

%% plot velocity

subplot(2, 2, 2);
hold off
plot(time_plot, data(plot_window, velocity), 'r--', 'LineWidth', 1.5);
hold on
plot(time_plot, velocity_est, 'b', 'LineWidth', 1.5);
title('Velocity');
legend('Measured velocity', 'Estimated in open-loop');
xlabel('Time [s]');
ylabel('Velocity [m/s]');
axis([0 time_plot(end) -1 2]);

%% plot acceleration

subplot(2, 2, 3);
hold off
plot(time_identifikace, acceleration_fit, 'r--', 'LineWidth', 1.5);
hold on
plot(time_plot, acceleration_est, 'b', 'LineWidth', 1.5);
title('Acceleration');
legend('Derived velocity', 'Estimated in open-loop');
xlabel('Time [s]');
ylabel('Acceleration [m/s^2]');
axis([0 time_plot(end) -1.2 2.1]);

%% plot input

subplot(2, 2, 4);
hold off
plot(time_plot, data(plot_window, input), 'b', 'LineWidth', 1.5);
hold on
title('Input');
legend('Input, desired angle of attitude');
xlabel('Time [s]');
ylabel('Input [-]');
axis([0 time_plot(end) -450 800]);  

set(hFig, 'Units', 'centimeters');
set(hFig, 'Position', [0 0 21 21*0.5625])

tightfig(hFig);

%% plotting for thesis

hFig = figure(2);

subplot(1, 2, 1);   
hold off
plot(time_plot, data(plot_window, velocity), 'r', 'LineWidth', 1.5);
hold on
plot(time_identifikace, velocity_fit, 'b', 'LineWidth', 1.5);
title('Velocity');
legend('Velocity', 'Fitted polynomial');
xlabel('Time [s]');
ylabel('Velocity [m/s]');
axis([0 time_plot(end) -1 2]);

subplot(1, 2, 2);
hold off
plot(time_plot, data(plot_window, input), 'b', 'LineWidth', 1.5);
hold on
title('Input');
legend('Input, desired angle of attitude');
xlabel('Time [s]');
ylabel('Input [-]');
axis([0 time_plot(end) -450 800]);  

set(hFig, 'Units', 'centimeters');
set(hFig, 'Position', [0 0 21 21*0.5625/2])

drawnow;

pause(2);

tightfig(hFig);

%% Kalman test

covariance = eye(3);

kalmanfrom = from+2800;
kalmanto = kalmanfrom+800;

dt = 0.0114;

kalmandata = data(kalmanfrom:kalmanto, :);
timekalman = integrate(kalmandata(:, 1));

states(:, 1) = [0; kalmandata(1, velocity); 0];

A = [1 dt 0;
     0 1 dt;
     0 0 0.9799];
 
B = [0; 0; 5.0719e-05];

R = [1 0 0;
     0 0.0000053333 0;
     0 0 0.01];
 
Q = diag([0.08]);

C = [0, 1, 0];

exp_filter(1) = kalmandata(1, velocity);
exp_alpha = 0.90;

for i=2:size(kalmandata, 1)
    [states(:, i) covariance] = kalman(states(:, i-1), covariance, kalmandata(i, velocity), kalmandata(i, input), A, B, R, Q, C);
    exp_filter(i) = exp_alpha*exp_filter(i-1) + (1-exp_alpha)*kalmandata(i, velocity);
end

%% 

hFig = figure(3);
 
hold off
plot(timekalman, kalmandata(:, velocity), 'r', 'LineWidth', 1.5);
hold on
plot(timekalman, exp_filter, 'g', 'LineWidth', 1.5);
plot(timekalman, states(2, :), 'b', 'LineWidth', 1.5);
legend('Measured velocity', 'Exponential filter', 'Kalman filter');
xlabel('Time [s]');
ylabel('Velocity [m/s]');
axis([0 timekalman(end) -0.9 1.1]);

set(hFig, 'Units', 'centimeters');
set(hFig, 'Position', [0 0 21 21*0.5625/2])

drawnow;

pause(2);

tightfig(hFig);
