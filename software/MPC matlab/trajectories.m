%% circle

dt = 0.0114;

traj = load('trajectory_uav_coord20150806T123529.txt');

to = size(traj, 1)-500;

koef = 1;

elevator_pos = [zeros(500, 1); traj(1:to, 2)./koef];
aileron_pos = [zeros(500, 1); -traj(1:to, 1)./koef];

elevator_pos = [elevator_pos; elevator_pos(end)*ones(500, 1)];
aileron_pos = [aileron_pos; aileron_pos(end)*ones(500, 1)];

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

%%

fid = fopen('trajectories.c', 'w');

fprintf(fid, '#include "trajectories.h"\n');
fprintf(fid, '#include <avr/pgmspace.h>');

fprintf(fid, '\n\n');

printMatrixC(fid, 'const float elevatorCircle[TRAJECTORY_CIRCLE_LENGTH] PROGMEM', '%3.3f', elevator_pos);

printMatrixC(fid, 'const float aileronCircle[TRAJECTORY_CIRCLE_LENGTH] PROGMEM', '%3.3f', aileron_pos);

fclose(fid);