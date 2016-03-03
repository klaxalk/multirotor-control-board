%najde periodu senzoru výšky
clear
data = load('LOG-9658.TXT');
readings = [integrate(data(:, 1)), data(:, 4)];
lasttime = readings(1, 1);
lastvalue = readings(1, 2);
periodcnt = 0;
periods = zeros(length(readings), 1);

for n = 1:1:length(readings)
    if ~(lastvalue == readings(n, 2))
        lastvalue = readings(n, 2);
        periodcnt = periodcnt + 1;
        periods(periodcnt) = readings(n, 1) - lasttime;
        lasttime = readings(n, 1);
    end
end
periods = periods(1:periodcnt);

hist(periods, 500);
title('Histogram of sensor interval');
xlabel('Occurrences [-]');
ylabel('Interval [s]');