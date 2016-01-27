clear

data = load('LOG-9658.TXT');

%% definice sloupcu v datechø

dt = 1;
elevatorSpeed = 2;
aileronSpeed = 3;
altitude = 4;
elevatorControl = 5;
aileronControl = 6;
altitudeControl = 7;
bateryLevel = 8;

%% vyrez

from = 1;
to = length(data);

%% osa dopredu/dozadu

figure(1)

subplot(2, 1, 1);
plot(integrate(data(:, dt)), data(:, elevatorSpeed));
title('Elevator Speed');
xlabel('Time [s]');
ylabel('Speed [m/s]');

subplot(2, 1, 2);
plot(integrate(data(:, dt)), data(:, elevatorControl));
title('Elevator control action');
xlabel('Time [s]');
ylabel('Control action []');

%% osa doleva/doprava

figure(2)

subplot(2, 1, 1);
plot(integrate(data(:, dt)), data(:, aileronSpeed));
title('Aileron Speed');
xlabel('Time [s]');
ylabel('Speed [m/s]');

subplot(2, 1, 2);
plot(integrate(data(:, dt)), data(:, aileronControl));
title('Aileron control action');
xlabel('Time [s]');
ylabel('Control action []');

%% osa výšky

figure(3)

subplot(2, 1, 1);
plot(integrate(data(:, dt)), data(:, altitude));
title('Altitude');
xlabel('Time [s]');
ylabel('Altitude [m]');

subplot(2, 1, 2);
plot(integrate(data(:, dt)), data(:, altitudeControl));
title('Altitude control action');
xlabel('Time [s]');
ylabel('Control action []');

%% Baterie

figure(4);
plot(integrate(data(:, dt)), data(:, bateryLevel));
