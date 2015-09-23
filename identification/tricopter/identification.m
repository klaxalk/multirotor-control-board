clear all;

data = load('log.txt');

%% data labels

dt = 1;
velocity = 2;
altitude = 3;
input = 4;

%% data margins

% crop the data
from = 2350;
to = size(data, 1) - 1500;

% fiting from:to
plot_window = from:to;

% identification from:to
margin = 200;
iden_window = (from+margin):(to-margin); 

meanDt = mean(data(:, 1))

%% integrate dt to time vector

time_plot = integrate(data(plot_window, dt));
time_identifikace = integrate(data(iden_window, dt))+time_plot(margin);

%% position

% integrating velocity
position = integrate(data(plot_window, velocity), data(plot_window, dt)); 

%% velocity

% fitting velocity with polynome
velocity_pol = polyfit(time_plot', data(plot_window, velocity)', 110);
velocity_fit = polyval(velocity_pol, time_identifikace);

%% acceleration

% derive the velocity polynomial
acceleration_pol = polyder(velocity_pol);
acceleration_fit = polyval(acceleration_pol, time_identifikace);

%% input

% shift input of the offset
data(:, input) = extreme_values_filter(data(:, input)) - mean(data(plot_window(1:20), input));

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

%% plot velocity

subplot(2, 2, 2);
hold off
plot(time_plot, data(plot_window, velocity), 'r--', 'LineWidth', 1.5);
hold on
plot(time_plot, velocity_est, 'b', 'LineWidth', 1.5);
title('Speed');
legend('Measured speed', 'Estimated in open-loop');
xlabel('Time [s]');
ylabel('Speed [m/s]');

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

%% plot input

subplot(2, 2, 4);
hold off
plot(time_plot, data(plot_window, input), 'b', 'LineWidth', 1.5);
hold on
title('Input');
legend('Input, desired angle of attitude');
xlabel('Time [s]');
ylabel('Input [-]');
