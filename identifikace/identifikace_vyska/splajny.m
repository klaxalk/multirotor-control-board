clear
data = load('LOG-9658.TXT');


%spline and filter config
splinestart = 9;
splinestop = 200;
period = 0.05;
sensor_bottom = 0.4;
sensor_top = 1.6;
cut_pos = 0.1;
cut_neg = -0.1;

%data prepare and filter
X = splinestart:period:splinestop;
times = integrate(data(:, 1));
[Ay, Ax] = xlimit(data(:, 4), times, sensor_bottom, sensor_top);
[Ax, Ay] = derepetize(Ax, Ay);

%plot filtered data and fitted splines
subplot(3, 1, 1)
hold on;
%plot(times, data(:, 4));
[Ax, Ay] = cutfilter(Ax, Ay, cut_pos, cut_neg);
%plot(Ax, Ay, 'r');

%create splines
p1 = csaps(Ax, Ay, 0.995);
pc = csaps(times, data(:, 7), 0.996);
pv = csaps(times, data(:, 8), 0.990);
[breaks,coefs1,l,k,d] = unmkpp(p1);

%plot splines
plot(X, ppval(p1, X), 'b');
hold off;
title('Altitude');
xlabel('Time [s]');
ylabel('Altitude [m]');
axis([90 160 0.25 1.75]);

%plot the splines' 1st derivative
subplot(3, 1, 2);
coefs2 = zeros(size(coefs1, 1), 3);
for n = 1:1:size(coefs1, 1)
    coefs2(n,:) = polyder(coefs1(n,:));
end
p2 = mkpp(breaks,coefs2);
hold on;
plot(X, ppval(p2, X), 'b');
hold off;
title('Speed');
xlabel('Time [s]');
ylabel('Speed [m/s]');
axis([90 160 -1.5 1.5]);

%plot the splines' 2nd derivative
subplot(3, 1, 3);
coefs3 = zeros(size(coefs1, 1), 2);
for n = 1:1:size(coefs2, 1)
    coefs3(n,:) = polyder(coefs2(n,:));
end
p3 = mkpp(breaks,coefs3);
hold on;
%plot(times, (data(:, 7)/100) + (data(:, 8)*2.3) -92.4);
plot(X, ppval(p3, X), 'b');
hold off;
title('Acceleration');
xlabel('Time [s]');
ylabel('Acceleration [m/s^2]');
axis([90 160 -3.5 3.5]);

%values for identification
altit = [Ax Ay];
speed = [X; ppval(p2, X)]';
accel = [X; ppval(p3, X)]';
altit_unfil = [times data(:, 4)];
cont = [times data(:, 7)];
volt = [times data(:, 8)];
sensor_residuals = [times (data(:, 4) - ppval(p1, times))];
sensor_residuals = sensor_residuals(31:33700, :);
accel_ident = ppval(p3, X)';
cont_ident = ppval(pc, X)';
volt_ident = ppval(pv, X)';
const_ident = ones(size(X))';