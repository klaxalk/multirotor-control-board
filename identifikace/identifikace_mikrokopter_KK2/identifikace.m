clear all;

data = load('log6.txt');

%% data labels

dt = 1;
velocity = 2;
altitude = 3;
input = 4;

%% data margins

% okrojení dat
from = 700;
to = size(data, 1)-200;

% od-do fitování
plot_window = from:to;

% od-do identifikace
margin = 180;
iden_window = (from+margin):(to-margin); 

%% integrate dt to time vector

time_plot = integrate(data(plot_window, dt));
time_identifikace = integrate(data(iden_window, dt))+time_plot(margin);

%% position

% integruju rychlost
position = integrate(data(plot_window, velocity), data(plot_window, dt)); 

%% velocity

% fituji rychlost polynomem
velocity_pol = polyfit(time_plot', data(plot_window, velocity)', 140);
velocity_fit = polyval(velocity_pol, time_identifikace);

%% acceleration

% derive the velocity polynomial
acceleration_pol = polyder(velocity_pol);
acceleration_fit = polyval(acceleration_pol, time_identifikace);

%% input

% shift input of the offset
data(:, input) = data(:, input) - mean(data(plot_window(1:20), input));

%% identify transfare input -> acceleration
input_window = data(iden_window, input);
dt_window = data(iden_window, dt);
meandt = mean(dt_window)

B = acceleration_fit(2:end);
A = [acceleration_fit(1:end-1) input_window(1:end-1)];

P = A\B;
P0 = P(1)
P1 = P(2)

%% Estimating all values from input

acceleration_est = firstOrder(data(plot_window, input), P);

velocity_est = integrate(acceleration_est, data(plot_window, dt));

position_est = integrate(velocity_est, data(plot_window, dt));

%% compute noise of velocity 

noise = std(velocity_fit - data(iden_window, velocity))

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
plot(time_identifikace, velocity_fit, 'black', 'LineWidth', 1.5);
title('Speed');
legend('Measured speed', 'Estimated in open-loop');
xlabel('Time [s]');
ylabel('Speed [m/s]');
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
% axis([0 time_plot(end) -450 800]);  



