clear all

a = load('LOG-9915.TXT');

from = 101;
to = size(a, 1);

mpcEnabled = a(:, 17);

a(:, 1) = a(:, 1).*mpcEnabled;
a(:, 2) = a(:, 2).*mpcEnabled;

speedLimit = 0.35;

dt = 0.033;
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

%% Wind estimate

wind = (a(:, 6).^2 + a(:, 5).^2).^(0.5)*0.545;
figure(3);

plot(time, wind(from:to), 'b');

%% 

fig = figure(4);
set(gcf, 'Color', 'white')

nFrames = length(from:to);

set(gca, 'nextplot','replacechildren', 'Visible', 'off');

%# preallocate
mov(1:nFrames) = struct('cdata',[], 'colormap',[]);

span = 100;

for k=1:nFrames
    
    %% plot position in 3D     
%     hold off
%     plot3(a(from+k-span:from+k-1, 1), a(from+k-span:from+k-1, 2), ones(1, span)*0.75, 'b', 'LineWidth', 3);
%     hold on
%     plot3(a(from+k-span:from+k-1, 7), a(from+k-span:from+k-1, 8), ones(1, span)*0.75, 'r', 'LineWidth', 3);
% 
%     scatter3(a(from+k-1, 1), a(from+k-1, 2), 0.75, 60, 'b', 'filled');
%     scatter3(a(from+k-1, 7), a(from+k-1, 8), 0.75, 60, 'r', 'filled');
%     
%     view([185 21]);
% 
%     xlabel('X [m]');
%     ylabel('Y [m]');
%     zlabel('Z [m]');
%     legend('Estimated position', 'Setpoint', 'Location', 'northwest');
%     
%     grid;
%     axis([-0.6 0.6 -0.6 0.6 0 2]);

    %% plot wind force     
    hold off
    plot(0:0.039:(span-1)*0.039, wind(from+k-span:from+k-1));
    hold on
    title('Estimated external force');
    ylabel('Force [N]');
    xlabel('Time [s]');
    grid;
    axis([0 (span-1)*0.039 0 0.4]);
    
    set(fig, 'OuterPosition', [0 0 720 360]);

    %% draw it and save the frame     
    
    drawnow;
    
    mov(k) = getframe(gcf);
end

close(gcf)

movie2avi(mov, 'wind2.avi', 'compression', 'None', 'fps', 30);

% potom je nutné to prohnad ffmpeg s -q 0

