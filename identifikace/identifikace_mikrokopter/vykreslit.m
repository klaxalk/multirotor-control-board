clear all

data = load('let1_23.09.2013.txt');

%% definice sloupcu v datech

pitch = 1;
roll = 2;
elevator = 3;
aileron = 4;
throttle = 5;
x = 6;
y = 7;
yaw = 8;
x_dot = 9;
y_dot = 10
z = 11;
time = 12;

%% vyrez

from = 65;
to = 1500;

len = to-from+1;

cut = 60;

from_cut = 65+cut;
to_cut = to-cut;

len_cut = to_cut - from_cut + 1;

%% plotting osa dopredu/dozadu

x = filtr_extremnich_hodnot(data(from:to, x));
dx = filtr_extremnich_hodnot(data(from:to, x_dot));
phi = filtr_extremnich_hodnot(data(from:to, pitch));
phi_d = data(from:to, elevator);

% fitting dx
p_dx = polyfit(1:len, dx', 40);
dx_fit = polyval(p_dx, 1:len);

% fitting phi_d
p_phi_d = polyfit(1:len, phi_d', 40);
phi_d_fit = polyval(p_phi_d, 1:len);

% derive dx
p_dx_der = polyder(p_dx);
dx_der_fit = polyval(p_dx_der, 1:len);

%% identify the px4flow noise

std(dx - dx_fit')

%% identify the transfer between phi_d, phi

A(:, 1) = phi(cut:end-cut-1);
A(:, 2) = phi_d_fit(cut+1:end-cut)'.*0.016;% .*(data(from_cut:to_cut, time)-data(from_cut-1:to_cut-1, time));
B = phi(cut+1:end-cut);
P = A\B;
P0 = P(1, 1)
P1 = P(2, 1)

clear A
clear P
clear B

%% identify the transfer between phi_d, phi

A(:, 1) = dx_fit(cut:end-cut-1)';
A(:, 2) = phi(cut+1:end-cut).*0.016;% .*(data(from_cut:to_cut, time)-data(from_cut-1:to_cut-1, time));
B = dx_fit(cut+1:end-cut)';
P = A\B;
P2 = P(1, 1)
P3 = P(2, 1)

%% 

figure(1)
subplot(4, 1, 1);
plot(data(from:to, time), x);
title('Poloha v ose X');
xlabel('Cas [s]');
ylabel('Poloha []');

subplot(4, 1, 2);
hold off
plot(data(from:to, time), dx);
hold on
plot(data(from_cut:to_cut, time), dx_fit(cut:end-cut-1), 'r');
title('Rychlost v ose X');
xlabel('Cas [s]');
ylabel('Rychlost [m/s]');

subplot(4, 1, 3);
hold off
plot(data(from:to, time), phi);
hold on
plot(data(from_cut:to_cut, time), dx_der_fit(cut:end-cut-1).*500, 'r');
title('Naklon v ose X');
xlabel('Cas [s]');
ylabel('Uhel [deg]');

subplot(4, 1, 4);
hold off
plot(data(from:to, time), phi_d);
hold on
plot(data(from_cut:to_cut, time), phi_d_fit(cut:end-cut-1), 'r');
title('Ridici signal v ose X (pozadovany naklon)');
xlabel('Cas [s]');
ylabel('Uhel [deg]');

