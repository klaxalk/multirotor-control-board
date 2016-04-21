% This script generates simple 2D trajectories for development and testing of
% MPC embedded control. It generates ANSI C file which should be included
% in the xMega source project.

% what sampling time to use?
dt = 0.0101;

% fly with speed ()..
speed = 1.5;

k = 0.01;

elevator_vel = [zeros(1, round(3/dt)) speed*ones(1, round(4/dt)) zeros(1, round(6/dt)) -speed*ones(1, round(4/dt)) zeros(1, round(3/dt))];
aileron_vel = [zeros(length(elevator_vel), 1)];

elevator_pos = integrate(elevator_vel, dt);
aileron_pos = integrate(aileron_vel, dt);

% plot the created 2D trajectory 

figure(20);

subplot(2, 1, 1);   
hold off
plot(integrate(ones(1, length(elevator_pos)).*dt), elevator_pos);
hold on
plot(integrate(ones(1, length(aileron_pos)).*dt), aileron_pos, 'r');
title('Positions');
xlabel('Time [s]');
ylabel('Position [m]');

subplot(2, 1, 2);
scatter(-aileron_pos, elevator_pos);
axis equal;
xlabel('X [m]');
ylabel('Y [m]');
title('Top-down view');

%% generate the ANSI C file with the trajectories

fid = fopen('trajectories.c', 'w');

fprintf(fid, '#include "trajectories.h"\n');
fprintf(fid, '#include <avr/pgmspace.h>');

fprintf(fid, '\n\n');

fprintf(fid, '// length = %d', length(elevator_pos));

fprintf(fid, '\n\n');

printMatrixC(fid, 'const float trajectoryElevator[TRAJECTORY_LENGTH] PROGMEM', '%3.3f', elevator_pos);

printMatrixC(fid, 'const float trajectoryAileron[TRAJECTORY_LENGTH] PROGMEM', '%3.3f', aileron_pos);

fclose(fid);