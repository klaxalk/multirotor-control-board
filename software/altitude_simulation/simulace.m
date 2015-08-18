dt = 0.0114;

g = 9.8;

A = [1 dt 0 0 0;
     0 1 dt 0 0;
     0 0 0 1 1;
     0 0 0 0.9712 0;
     0 0 0 0 0];
 
B = [0 0;
     0 0;
     0 0;
     0.0004 0;
     0 -g];

x(:, 1) = [1; 0; 0; 0; 0];
 
u = [0; 1];

simuLen = round(20/dt);

timeVec = 0:dt:simuLen*dt - dt;

setPoint = 1;

P = 10;
I = 10;
D = 100000;
integral = 0;
lastError = 0;
antiwindup = 1000;

for i=2:simuLen
    
    [u(1, 1), integral, lastError] = pidController(setPoint - x(1, i-1), lastError, integral, P, I, D, antiwindup);
    
    % system step
    x(:, i) = A*x(:, i-1) + B*u;
    
    % limit the minimal altitude
%     if x(1, i) < 0
%        x(1:2, i) = 0;
%     end
    
end

figure(1);
subplot(2, 1, 1);
plot(timeVec, x(1, :));
axis([0 timeVec(end), -5, 5]);
