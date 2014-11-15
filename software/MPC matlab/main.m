clear all;

%% Matice systemu

A = [1 1 0;
     0 1 0.0058;
     0 0 0.9744];
 
B = [0; 0; 0.0221];

% pocet stavu systemu 
n_states = size(A, 1);

% pocet vstupu do systemu
u_size = size(B, 2);

%% Pomocne promenne

% delka predikcniho horizontu
horizon_len = 140;

%% Matice A_roof
% n = delka predikcniho horizontu
% A_roof = [A;
%           A^2;
%           A^3;
%           ...;
%           A^n]; 

A_roof = zeros(horizon_len * n_states, n_states);

for i=1:horizon_len
   A_roof(((i-1)*n_states+1):(n_states*i), :) = A^i; 
end

%% Matice B_roof
% n = delka predikcniho horizontu
% B_roof = [B,        0,        0,   0;
%           AB,       B,        0,   0;
%           A^2B,     AB,       B,   0;
%           ...;
%           A^(n-2)B, A^(n-1)B, ..., 0;
%           A^(n-1)B, A^(n-2)B, ..., B;

B_roof = zeros(horizon_len * n_states, horizon_len);

for i=1:horizon_len
    for j=1:i
        B_roof((i-1)*n_states+1:i*n_states, ((j-1)*u_size+1):j*u_size) = (A^(i - j))*B;
    end
end

%% Matice Q_roof
% n = pocet stavu
% Q = n*n
%     diagonala, penalizace odchylek stavu
% S = n*n
%     penalizace odchylky posledniho stavu
% Q_roof = [Q,   0,   ...,  0;
%           0,   Q,   ...,  0;
%           ..., ..., Q,    0;
%           0,   ..., ...,  S];

Q = [1, 0, 0;
     0, 0, 0;
     0, 0, 0];

S = [100, 0, 0;
     0, 0, 0;
     0, 0, 0];
 
Q_roof = zeros(n_states * horizon_len, n_states * horizon_len);

for i=1:horizon_len
    Q_roof((i-1)*n_states+1:i*n_states, (i-1)*n_states+1:i*n_states) = Q; 
end

Q_roof(end-n_states+1:end, end-n_states+1:end) = S;

%% Matice P_roof
% penalizace akcnich zasahu
% P_roof = [P,   0,   ...,  0;
%           0,   P,   ...,  0;
%           ..., ..., P,    0;
%           0,   ..., ...,  P];

P = 10;

P_roof = zeros(horizon_len, horizon_len);

for i=1:horizon_len
    P_roof(i, i) = P;
end

%% Matice H
% matice kvadratickeho clenu kvadraticke formy QP

H = B_roof'*Q_roof*B_roof + P_roof;
H_inv = (0.5*H)^(-1);

%% øídicí smyèka s MPC

% delka simulace
simu_len = 1000;

% pozadovana pozice
x_ref = zeros(n_states*simu_len*2, 1);
x_ref((n_states*simu_len)/2+1:3:end, 1) = 1000+200*sin(linspace(1, 50, length((n_states*simu_len)/2+1:3:length(x_ref))))';

% pocatecni podminky simulace
x(:, 1) = [1000; 0; 0];

noisy_hist = [0; 0; 0];

% vektor akcnich zasahu
u = zeros(horizon_len, u_size);

% saturace akcnich zasahu
saturace = 150;

for i=2:simu_len
    
    
    noise = [randn*5.8*1; 0; 0];
    
    noisy_hist(:, i-1) = x(:, i-1)+noise;
    
    X_0 = A_roof*((x(1:3, i-1)+noise)) - x_ref(n_states*(i-1)+1:n_states*(i-1)+horizon_len*n_states, 1);
    c = (X_0'*Q_roof*B_roof)';

    u_cf = H_inv*(c./(-2));
    
    % saturuj akcni zasah     
    u_sat = u_cf(1);
    
    if (u_sat > saturace)
        u_sat = saturace;
    elseif (u_sat < -saturace)
        u_sat = -saturace;
    end
        
    % spocti nove stavy podle modelu     
    x(:, i) = A*x(:, i-1) + B*u_sat;
    
    % simulace predikce z aktualniho vektoru akcnich zasahu
    x_cf(:, 1) = x(:, i-1)+noise;
    for j=2:horizon_len
        x_cf(:, j) = A*x_cf(:, j-1) + B*u_cf(j-1);
    end
    
    figure(1);
    subplot(1, 2, 1);
    hold off
    plot(x_ref(1:3:n_states*simu_len)./1000, 'r');
    hold on
    plot(x(1, 1:i)./1000, 'b');
    plot(noisy_hist(1, 1:(i-1))./1000, '--b');
    title('Position in time');
    ylabel('Position [m]');
    plot(i:(i+length(x_cf)-1), x_cf(1, :)./1000, '--r');
    
    subplot(1, 2, 2);
    plot(u_cf./10, 'r');
    title('Predicted action');
    ylabel('Desired angle [deg]');
    axis([0, horizon_len, -saturace/10, saturace/10]);
    
end