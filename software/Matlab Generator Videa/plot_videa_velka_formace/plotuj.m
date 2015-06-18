clear all

a = load('LOG-9764.TXT');

dt = 0.033;

from = round(20/dt);
to = round(75/dt);

mpcEnabled = a(:, 17);

a(:, 1) = a(:, 1).*mpcEnabled;
a(:, 2) = a(:, 2).*mpcEnabled;

speedLimit = 0.35;

time = integrate(ones(1, length(from:to)).*dt);

figure(1)
subplot(3, 1, 1);
hold off
plot(time, a(from:to, 1), 'b');
hold on
plot(time, a(from:to, 7), 'r');

% axis([0 time(end) -2.5 2.5]);
title('Elevator position');
xlabel('Time [s]');
ylabel('Position [m]');
legend('Estimated position', 'Setpoint');

subplot(3, 1, 2);
hold off
plot(time, a(from:to, 9), 'r');
hold on
plot(time, a(from:to, 3), 'b');
plot(time, ones(1, length(from:to)).*-speedLimit, 'color', 'black');
plot(time, ones(1, length(from:to)).*speedLimit, 'color', 'black');
axis([0 time(end) -2 2]);
title('Elevator speed');
xlabel('Time [s]');
ylabel('Speed [m/s]');
legend('Measured speed', 'Estimated Speed', 'Speed limit');

subplot(3, 1, 3);
hold off
plot(time, a(from:to, 5), 'b');
axis([0 time(end) -0.2 0.8]);
title('Estimated acceleration error (wind disturbance)');
xlabel('Time [s]');
ylabel('Acceleration error [m/s^2]');
legend('Estimated acceleration error');

figure(2)

subplot(3, 1, 1);
hold off
plot(time, a(from:to, 2), 'b');
hold on
plot(time, a(from:to, 8), 'r');
title('Aileron position');
xlabel('Time [s]');
ylabel('Position [m]');
legend('Estimated position', 'Setpoint');

subplot(3, 1, 2);
hold off
plot(time, a(from:to, 10), 'r');
hold on
plot(time, a(from:to, 4), 'b');
plot(time, ones(1, length(from:to)).*-speedLimit, 'color', 'black');
plot(time, ones(1, length(from:to)).*speedLimit, 'color', 'black');
axis([0 time(end) -0.6 0.6]);
title('Aileron speed');
ylabel('Speed [m/s]');
xlabel('Time [s]');
legend('Measured speed', 'Estimated Speed', 'Speed limit');

subplot(3, 1, 3);
hold off
plot(time, a(from:to, 6), 'b');
axis([0 time(end) -0.2 0.8]);
title('Estimated acceleration error (wind disturbance)');
xlabel('Time [s]');
ylabel('Acceleration error [m/s^2]');
legend('Estimated acceleration error');

%%

% figure(5);
% 
% hold off
% 
% for i=from:from+length(from:to)-201
%    
%     step = (a(i+200, 7) - a(i, 7))/200;
%     
%     u = a(i, 7);
%     
%     for j=2:200
%         u(j) = u(j-1) + step;
%     end
%     
%     hold off
%     plot(i-from+1:i-from+200, u, 'b');
%     hold on
%     plot(a(from:to, 7));
%     plot(a(from:to, 1), 'r');
%     
%     drawnow;
% end
% 
% plot(a(from:to, 7), 'r');
% plot(a(from:to, 1), 'r');

%% 3D plot

hFig = figure(11);
set(gcf, 'Color', 'white')
set(gca, 'nextplot','replacechildren', 'Visible', 'off');
    
hold off
plot3(a(from:to, 1), a(from:to, 2), ones(1, length(from:to))*0.75, 'b', 'LineWidth', 3);
hold on
plot3(a(from:to, 7), a(from:to, 8), ones(1, length(from:to))*0.75, 'r', 'LineWidth', 3);

grid;

axis([-0.2 6.2 -1.6 0.2 0 1.5]);

axis equal;

xlabel('X [m]');
ylabel('Y [m]');
zlabel('Z [m]');
legend('Estimated position', 'Setpoint', 'Location', 'north');

view([-97 11]);

%% 
% 
% fig = figure(4);
% set(gcf, 'Color', 'white')
% 
% nFrames = length(from:to);
% 
% set(gca, 'nextplot','replacechildren', 'Visible', 'off');
% 
% %# preallocate
% mov(1:nFrames) = struct('cdata',[], 'colormap',[]);
% 
% span = 100;
% 
% for k=1:nFrames
%     
% %     ones(1, length(from:from+k-1))*0.75
%     
%     hold off
%     plot3(a(from:from+k-1, 7), a(from:from+k-1, 8), ones(1, length(from:from+k-1))*0.75, 'r', 'LineWidth', 3);
%     hold on
%     plot3(a(from:from+k-1, 1), a(from:from+k-1, 2), ones(1, length(from:from+k-1))*0.75, 'b', 'LineWidth', 3);
% 
%     scatter3(a(from+k-1, 7), a(from+k-1, 8), 0.75, 60, 'r', 'filled');
%     scatter3(a(from+k-1, 1), a(from+k-1, 2), 0.75, 60, 'b');
% 
%     scatter3(6.2, -1.5, 0, 1, 'b', 'filled');
%     scatter3(6.2, 0.2, 0.75, 1, 'b', 'filled');
%     scatter3(-0.2, -1.5, 1.5, 1, 'b', 'filled');
%     scatter3(-0.2, 0.2, 0.75, 1, 'b', 'filled');
%     
%     xlabel('X [m]');
%     ylabel('Y [m]');
%     zlabel('Z [m]');
%     legend('Setpoint', 'Estimated position', 'Location', 'north');
%     grid;
%     
%     axis equal;
%     
%     view([-97 11]);
%     
%     set(fig, 'OuterPosition', [0 0 480 720]);
% 
%     drawnow;
%     
%     mov(k) = getframe(gcf);
% end
% 
% close(gcf)
% 
% movie2avi(mov, 'final_prvni.avi', 'compression', 'None', 'fps', 30);

