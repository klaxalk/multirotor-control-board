%% Boèní/Pøední systém 

dt = 0.0114; 

len = 1000;

A = [1 dt   0  0            0;  % pozice
     0 1    dt 0            0;  % rychlost
     0 0    0  1            1;  % skuteèné zrychlení
     0 0    0  0.9796       0;  % normální zrychlení
     0 0    0  0            1]; % chyba zrychlení
 
B = [0; 0; 0; 0.0045*dt; 0];

x(1:5, 1) = [1; 0; 0; 0; 0];

u = -10;

for i=2:len
   
    x(:, i) = A*x(:, i-1) + B*u;
    
end

plot(0:dt:dt*(len-1), x(1, :));

%% Výškový system

clear all;

P1 = 0.9658;
P2 = 0.0023;
g = 9.8;

dt = 0.0114;

A = [1 dt 0 0;  % výška
     0 1 dt 0;  % rychlost 
     0 0 0 1;   % skuteèné zrychlení
     0 0 0 P1]; % normální zrychlení (od motorù)
 
% vstup è.1 je skuteèný øídicí signál
% vstup è.2 je "workaround" pro gravitaci, musí být vždy 1
B = [0 0;
     0 0;
     0 -g;
     P2 0];
 
x(1:4, 1) = [1; 0; 0; 0];

u = [300; 1];

len = 200;

setpoint = 1;

Kp = 100000   ;
Kd = 100;

lasterror = 0;

for i=2:len
   
    error = setpoint - x(1, i-1);
    
    u(1, 1) = error*Kp;
    
    lasterror = error;
    
    x(:, i) = A*x(:, i-1) + B*u;
    
end

plot(0:dt:dt*(len-1), x(1, :));