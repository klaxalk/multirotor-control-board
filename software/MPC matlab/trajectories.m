%% circle
clear all

dt = 0.0101;

k = 0.0025;
speed = 0.25;

% osmicka
% elevator_vel = [speed*sin(0:k:2*pi) -speed*sin(0:k:2*pi)];
% aileron_vel = [-speed*cos(0:k:2*pi) -speed*cos(0:k:2*pi)];

elevator_vel = [zeros(1, 600) speed*ones(1, 2000) zeros(1, 600) -speed*ones(1, 2000) zeros(1, 600)];
aileron_vel = [elevator_vel.*0];

% aileron_vel = [zeros(1, 600) -speed*ones(1, 600) speed*ones(1, 400) -speed*ones(1, 400) speed*ones(1, 400) -speed*ones(1, 400) speed*sin(0:k:pi) zeros(1, 600)];
% elevator_vel = aileron_vel.*0;

elevator_pos = integrate(elevator_vel, dt);
aileron_pos = integrate(aileron_vel, dt);

figure(20);
subplot(2, 1, 1);
plot(elevator_pos);
subplot(2, 1, 2);
plot(aileron_pos);

% elevator_vel = 0.25*cos(0:0.006:2*pi);
% aileron_vel = 0.25*sin(0:0.006:2*pi);
% 
% elevator_pos = integrate(elevator_vel, dt);
% aileron_pos = integrate(aileron_vel, dt);
% aileron_pos = aileron_pos - mean(aileron_pos);

figure(1);
subplot(3, 1, 1);
% hold off
% plot(integrate(ones(1, length(elevator_vel)).*dt), elevator_vel);
% hold on
% plot(integrate(ones(1, length(aileron_vel)).*dt), aileron_vel, 'r');
% title('Velocities');

subplot(3, 1, 2);   
hold off
plot(integrate(ones(1, length(elevator_pos)).*dt), elevator_pos);
hold on
plot(integrate(ones(1, length(aileron_pos)).*dt), aileron_pos, 'r');
title('Positions');

subplot(3, 1, 3);
scatter(-aileron_pos, elevator_pos);
axis equal;

%%

fid = fopen('trajectoryCircle.c', 'w');

fprintf(fid, '#include "trajectoryCircle.h"\n');
fprintf(fid, '#include <avr/pgmspace.h>');

fprintf(fid, '\n\n');

fprintf(fid, '// length = %d', length(elevator_pos));

fprintf(fid, '\n\n');

printMatrixC(fid, 'const float elevatorCircle[TRAJECTORY_CIRCLE_LENGTH] PROGMEM', '%3.3f', elevator_pos);

printMatrixC(fid, 'const float aileronCircle[TRAJECTORY_CIRCLE_LENGTH] PROGMEM', '%3.3f', aileron_pos);

fclose(fid);