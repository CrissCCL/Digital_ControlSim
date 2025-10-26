clear all; close all; clc

%% Discretization of First-Order Identified Model (Non-Parametric)
Ts = 0.1;
gp  = tf(20,[50 1]);           % First-order plant
gpd = c2d(gp,Ts,'zoh');        % Discrete plant

%% Discrete PI Controller (Tustin)
Kp = 0.8;
Ti = 9;
K0 = Kp + Kp*Ts/(2*Ti);
K1 = -Kp + Kp*Ts/(2*Ti);

%% Closed-Loop Simulation (Direct difference equation)
[num,den] = tfdata(gpd,'v');
t   = 0:Ts:60;
Ref = ones(1,length(t));

y1 = 0; u1 = 0; e1 = 0;   % Histories

for k = 1:length(t)
    % ---- PLANT ----
    y(k) = num(2)*u1 - den(2)*y1;

    % ---- FEEDBACK ----
    e  = Ref(k) - y(k);
    u  = u1 + K0*e + K1*e1;

    % ---- SATURATION ----
    if u > 100
        u = 100; 
    end
    if u < 0
        u = 0;   
    end

    % ---- UPDATE STATES ----
    y1 = y(k);
    u1 = u;
    e1 = e;

    Usim(k) = u;
end

%% PLOTS
subplot(2,1,1)
plot(t,y,'+',t,Ref,'--','MarkerSize',4)
xlabel('Time [s]'), ylabel('Response')
legend('Simulation','Reference')

subplot(2,1,2)
plot(t,Usim,'+','MarkerSize',4)
xlabel('Time [s]'), ylabel('Control signal')
