## 🚀 DigitalControlSim

## 📖 Overview
This repository provides a simple **MATLAB/Octave tutorial** to simulate a discrete-time control system with saturation, illustrating the behavior you would expect in a microcontroller implementation.  
The tutorial focuses on:

- Discretizing a continuous-time plant
- Implementing a digital PI controller
- Adding saturation limits to emulate actuator constraints
- Visualizing closed-loop response

> ⚠️ **Note:** This is a simulation only. No hardware is involved.

## 📂 Contents
- `/code` → MATLAB/Octave scripts
- `/docs` → example plots

## 🔄 Simulation Setup
The example plant is:

$$
G(s) = \frac{20}{50 s + 1}
$$

Discretized using Zero-Order Hold (ZOH) with sampling period:

$$
T_s = 0.1 \text{ s}
$$

The discrete-time PI controller is implemented as:

$$
u[n] = u[n-1] + K_0 e[n] + K_1 e[n-1]
$$

With:

$$
K_0 = K_p + K_p \frac{T_s}{2 T_i}, \quad
K_1 = -K_p + K_p \frac{T_s}{2 T_i}
$$

Saturation limits are applied:

$$
0 \le u[n] \le 100
$$

---

## 🧪 Example Script

```matlab
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
```

Below are example plots generated with the script:

<p align="center">
<img width="500" alt="Control simulation" src="https://github.com/user-attachments/assets/f208de9a-0f46-4059-af58-50642f70e590" />
</p>

## 📜 License
MIT License
