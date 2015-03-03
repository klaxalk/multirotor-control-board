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

position = filtr_extremnich_hodnot(data(plot_window, altitude));
pos_offset = mean(position);
position = position - pos_offset;

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
action = data(plot_window, input) - mean(data(plot_window, input)) - 5;
action_pol = polyfit(time_plot', action, 80);
action_fit = polyval(action_pol, time_identifikace);

%% identify transfare input -> acceleration

B = acceleration_fit(2:end);
A = [acceleration_fit(1:end-1) action_fit(1:end-1)];

P = A\B;
P0 = P(1)
P1 = P(2)

%% Estimating all values from input

acceleration_est = firstOrder(action(margin:end-margin-1), P) + acceleration_fit(1);

velocity_est = integrate(acceleration_est, ones(1, length(iden_window)).*dt);

position_est = integrate(velocity_est, ones(1, length(iden_window)).*dt);

% %% plot position

hFig = figure(1);
subplot(2, 2, 1);
hold off
plot(time_identifikace, filtr_extremnich_hodnot(data(iden_window, altitude)), 'r--', 'LineWidth', 1.5);
hold on
plot(time_identifikace, position_est+pos_offset, 'b', 'LineWidth', 1.5);
title('Altitude');
legend('Measured altitude', 'Estimated in open-loop');
xlabel('Time [s]');
ylabel('Position [m]');
axis([time_identifikace(1) time_identifikace(end) 0.5, 1]);

%% plot velocity

subplot(2, 2, 2);
hold off
plot(time_identifikace, velocity_fit, 'r--', 'LineWidth', 1.5);
hold on
plot(time_identifikace, velocity_est, 'b', 'LineWidth', 1.5);
title('Velocity');
legend('Velocity (from altitude)', 'Estimated in open-loop');
xlabel('Time [s]');
ylabel('Velocity [m/s]');
axis([time_identifikace(1) time_identifikace(end) -0.3 1.1]);

%% plot acceleration

subplot(2, 2, 3);
hold off
plot(time_identifikace, acceleration_fit, 'r--', 'LineWidth', 1.5);
hold on
plot(time_identifikace, acceleration_est, 'b', 'LineWidth', 1.5);
title('Acceleration');
legend('Acceleration (from altitude)', 'Estimated in open-loop');
xlabel('Time [s]');
ylabel('Acceleration [m/s^2]');
axis([time_identifikace(1) time_identifikace(end) -1.2 3]);

%% plot input

subplot(2, 2, 4);
hold off
plot(time_plot, action, 'b', 'LineWidth', 1.5);
hold on
title('Input');
legend('Input');
xlabel('Time [s]');
ylabel('Input [-]');
axis([0 time_plot(end) -80 130]);  

set(hFig, 'Units', 'centimeters');
set(hFig, 'Position', [0 0 21 21*0.5625])

tightfig(hFig);