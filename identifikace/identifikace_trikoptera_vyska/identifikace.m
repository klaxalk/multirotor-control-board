clear all;

data = load('LOG-9910.TXT');

%% data labels

dt = 0.0114;
altitude = 1;
input = 2;

%% data margins

% okrojení dat
baf = 300;

from = 500 + baf;
to = 2000 + baf;

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

position_pol = polyfit(time_plot', position, 220);
position_fit = polyval(position_pol, time_identifikace);

%% velocity

% fituji rychlost polynomem
velocity_pol = polyder(position_pol);
velocity_fit = polyval(velocity_pol, time_identifikace);

%% acceleration

% derive the velocity polynomial
acceleration_pol = polyder(velocity_pol);
acceleration_fit = polyval(acceleration_pol, time_identifikace);

%% optimize over input offsets and find P

bestCrit = 99999999999999999999999999999999999;

for i=-1:0.005:0

    % shift input of the offset
    inputOffset = mean(data(plot_window, input));
    action = data(plot_window, input) - inputOffset + i;
    action_pol = polyfit(time_plot', action, 220);
    action_fit = polyval(action_pol, time_identifikace);

    B = acceleration_fit(2:end);
    A = [acceleration_fit(1:end-1) action_fit(1:end-1)];

    P = A\B;
    P0 = P(1);
    P1 = P(2);

    acceleration_est = firstOrder(action(margin:end-margin-1), P) + acceleration_fit(1);
    velocity_est = integrate(acceleration_est, ones(1, length(iden_window)).*dt) + velocity_fit(1);
    position_est = integrate(velocity_est, ones(1, length(iden_window)).*dt) + position_fit(1);
    
    crit = sum((acceleration_fit - acceleration_est).^2) + sum((position_fit - position_est).^2) + sum((velocity_fit - velocity_est).^2);

    if crit < bestCrit
       bestCrit = crit;
       bestP = P;
       bestI = i;
    end
end

%% simulate the open-loop for the found offset 

P = bestP

action = data(plot_window, input) - inputOffset + bestI;
action_pol = polyfit(time_plot', action, 200);
action_fit = polyval(action_pol, time_identifikace);

acceleration_est = firstOrder(action(margin:end-margin-1), P) + acceleration_fit(1);

velocity_est = integrate(acceleration_est, ones(1, length(iden_window)).*dt) + velocity_fit(1);

position_est = integrate(velocity_est, ones(1, length(iden_window)).*dt) + position_fit(1);

%% plot the altitude

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
axis([time_identifikace(1) time_identifikace(end) -0.2, 2.5]);

% plot velocity

subplot(2, 2, 2);
hold off
plot(time_identifikace, velocity_fit, 'r--', 'LineWidth', 1.5);
hold on
plot(time_identifikace, velocity_est, 'b', 'LineWidth', 1.5);
title('Velocity');
legend('Velocity (from altitude)', 'Estimated in open-loop');
xlabel('Time [s]');
ylabel('Velocity [m/s]');
axis([time_identifikace(1) time_identifikace(end) -0.9 2.7]);

% plot acceleration

subplot(2, 2, 3);
hold off
plot(time_identifikace, acceleration_fit, 'r--', 'LineWidth', 1.5);
hold on
plot(time_identifikace, acceleration_est, 'b', 'LineWidth', 1.5);
title('Acceleration');
legend('Acceleration (from altitude)', 'Estimated in open-loop');
xlabel('Time [s]');
ylabel('Acceleration [m/s^2]');
axis([time_identifikace(1) time_identifikace(end) -2.2, 4]);

% plot input

subplot(2, 2, 4);
hold off
plot(time_plot, action, 'b', 'LineWidth', 1.5);
hold on
title('Input');
legend('Input');
xlabel('Time [s]');
ylabel('Input [-]');
axis([0 time_plot(end) -145 150]);  

set(hFig, 'Units', 'centimeters');
set(hFig, 'Position', [0 0 21 21*0.5625])

tightfig(hFig);

%% plotting for thesis

hFig = figure(2);

subplot(1, 2, 1);   
hold off
plot(time_identifikace, filtr_extremnich_hodnot(data(iden_window, altitude)), 'r', 'LineWidth', 1.5);
hold on
plot(time_identifikace, position_fit+pos_offset, 'LineWidth', 1);
title('Altitude');
legend('Altitude', 'Fitted polynomial');
xlabel('Time [s]');
ylabel('Altitude [m]');
axis([time_identifikace(1) time_identifikace(end) 0.75, 1.8]);

subplot(1, 2, 2);
hold off
plot(time_plot, action, 'b', 'LineWidth', 1.5);
hold on
title('Input');
legend('Input');
xlabel('Time [s]');
ylabel('Input [-]');
axis([0 time_plot(end) -145 150]);  

set(hFig, 'Units', 'centimeters');
set(hFig, 'Position', [0 0 21 21*0.5625/2])

tightfig(hFig);