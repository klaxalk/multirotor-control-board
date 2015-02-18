%% circle

dt = 0.0114;

elevator_vel = 0.25*cos(0:0.003:2*pi);
aileron_vel = 0.25*sin(0:0.003:2*pi);

elevator_pos = integrate(elevator_vel, dt);
aileron_pos = integrate(aileron_vel, dt);
aileron_pos = aileron_pos - mean(aileron_pos);

figure(1);
subplot(3, 1, 1);
hold off
plot(integrate(ones(1, length(elevator_vel)).*dt), elevator_vel);
hold on
plot(integrate(ones(1, length(aileron_vel)).*dt), aileron_vel, 'r');
title('Velocities');

subplot(3, 1, 2);   
hold off
plot(integrate(ones(1, length(elevator_pos)).*dt), elevator_pos);
hold on
plot(integrate(ones(1, length(aileron_pos)).*dt), aileron_pos, 'r');
title('Positions');

subplot(3, 1, 3);
scatter(elevator_pos, aileron_pos);

%%

fid = fopen('trajectories.c', 'w');

fprintf(fid, '#include "trajectories.h"\n');
fprintf(fid, '#include <avr/pgmspace.h>');

fprintf(fid, '\n\n');

printMatrixC(fid, 'const float elevatorCircle[TRAJECTORY_CIRCLE_LENGTH] PROGMEM', '%3.3f', elevator_pos);

printMatrixC(fid, 'const float aileronCircle[TRAJECTORY_CIRCLE_LENGTH] PROGMEM', '%3.3f', aileron_pos);

fclose(fid);