clear all; close all; clc

%% Discretize the plant
Ts=0.1;
gp=tf(20,[50 1]);
gpd=c2d(gp,Ts,'zoh');

%% Discrete PI controller parameters
Kp=0.8;
Ti=9;
K0=Kp+Kp*Ts/(2*Ti);
K1=-Kp+Kp*Ts/(2*Ti);

%% Closed-loop simulation via direct integration
[num,den]=tfdata(gpd,'v');
t=0:Ts:60;
Ref=ones(1,length(t));
y1=0; u1=0; error1=0;

for i=1:length(t)
    % Process simulation
    y(i) = num(2)*u1 - den(2)*y1;

    % Control computation
    error = Ref(i) - y(i);
    Usim(i) = u1 + K0*error + K1*error1;

    % Saturation
    if Usim(i) > 100
        Usim(i) = 100;
    elseif Usim(i) < 0
        Usim(i) = 0;
    end

    % Update states
    y1 = y(i);
    u1 = Usim(i);
    error1 = error;
end

%% Plot results
subplot(2,1,1)
plot(t,y,'+',t,Ref,'--','MarkerSize', 4)
xlabel('Time [s]')
ylabel('Response')
legend('Sim','Ref')

subplot(2,1,2)
plot(t,Usim,'+','MarkerSize', 4)
ylabel('Control output')
xlabel('Time [s]')