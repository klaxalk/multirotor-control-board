clear all;

data = load('log.txt');

%% data labels

dt = 0.014;
altitude = 9;
input = 11;

%% data margins

% okrojení dat
from = 600;
to = 1400;

% od-do fitování
plot_window = from:to;

% od-do identifikace
margin = 200;
iden_window = (from+margin):(to-margin); 

%% integrate dt to time vector

time_plot = integrate(ones(1, length(plot_window)).*dt)';
time_identifikace = integrate(ones(1, length(iden_window))*dt) + dt*margin;

%% position

% integruju rychlost
position = filtr_extremnich_hodnot(data(plot_window, altitude));
position = position - mean(position);

position_pol = polyfit(time_plot', position, 150);
position_fit = polyval(position_pol, time_identifikace);

%% velocity

% fituji rychlost polynomem
velocity_pol = polyder(position_pol);
velocity_fit = polyval(velocity_pol, time_identifikace);

%% acceleration

% derive the velocity polynomial
acceleration_pol = polyder(velocity_pol);
acceleration_fit = polyval(acceleration_pol, time_identifikace);

%% input

% shift input of the offset
action = data(plot_window, input) - mean(data(plot_window, input)) - 3;
action_pol = polyfit(time_plot', action, 70);
action_fit = polyval(action_pol, time_identifikace);

%% identify transfare input -> acceleration

B = acceleration_fit(2:end);
A = [acceleration_fit(1:end-1).*dt action_fit(2:end).*dt];

P = A\B;
P0 = P(1)
P1 = P(2)

%% Estimating all values from input

acceleration_est = firstOrder(action(margin:end-margin-1), ones(1, length(iden_window)).*dt, P) + acceleration_fit(1);

velocity_est = integrate(acceleration_est, ones(1, length(iden_window)).*dt);

position_est = integrate(velocity_est, ones(1, length(iden_window)).*dt);

% %% plot position

figure(1);
subplot(2, 2, 1);
hold off
plot(time_plot, position, 'b');
hold on
plot(time_identifikace, position_fit, 'r');
title('Position');
legend('px4flow', 'estimated from input');
xlabel('time [s]');
ylabel('position [m]');

%% plot velocity

subplot(2, 2, 2);
hold off
plot(time_identifikace, velocity_fit, 'b');
hold on
plot(time_identifikace, velocity_est, 'g');
title('Velocity');
legend('px4flow', 'fitted polynome', 'estimated from input');
xlabel('time [s]');
ylabel('velocity [m/s]');

%% plot acceleration

subplot(2, 2, 3);
hold off
plot(time_identifikace, acceleration_fit, 'b');
hold on
plot(time_identifikace, acceleration_est, 'g');
title('Acceleration');
legend('estimate from px4flow', 'estimate from input');
xlabel('time [s]');
ylabel('acceleration [m/s^2]');

%% plot input

subplot(2, 2, 4);
hold off
plot(time_plot, action, 'b');
hold on
plot(time_identifikace, action_fit, 'r');
title('Input');
legend('input');
xlabel('time [s]');
ylabel('input []');
