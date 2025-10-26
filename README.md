# 🧪 Digital Control Simulation — First Order System + Saturation

This repository provides a **tutorial-oriented simulation** of a **digital PI control loop** applied to a **first-order system identified via non-parametric methods**.  
The objective is to reproduce in simulation the **same discrete behavior** expected when the controller is later implemented on a **microcontroller**, including **actuator saturation**.

## 🎯 Goals of the Tutorial

- Model a first-order process identified experimentally (non-parametric fit)
- Discretize the plant using Zero-Order Hold (ZOH)
- Implement a **discrete PI controller** using incremental form
- Add **saturation limits** to emulate real actuator constraints
- Compare reference tracking and control signal behavior

## 🧩 System Model

A first-order model without delay was identified experimentally from step-response data:

$$
G_p(s) = \frac{K}{\tau s + 1}
$$

In the included example:

$$
G_p(s)= \frac{20}{50s + 1}
$$

This model is discretized with sampling period  
$$T_s = 0.1\,s$$  
using Zero-Order Hold.


## ⚙️ Digital PI Controller (Incremental Form)

The discrete control law implemented is:

$$
u(k)=u(k-1)+K_0 e(k)+K_1 e(k-1)
$$

With tuning parameters derived from:
- Proportional gain: \(K_p\)
- Integral time: \(T_i\)
- Sampling time: \(T_s\)

Where

$$
K_0 = K_p + \frac{K_p}{2T_i}T_s
$$

$$
K_1 = -K_p + \frac{K_p}{2T_i}T_s
$$


## 🔒 Actuator Saturation

To emulate real microcontroller behavior — such as PWM range or fixed DAC limits —  
a **hard saturation** is enforced:

```matlab
if Usim(i) > 100
    Usim(i)=100;
end
if Usim(i) < 0
    Usim(i)=0;
end
```

Below are example plots generated with the script:

<p align="center">
<img width="500" alt="Control simulation" src="https://github.com/user-attachments/assets/f208de9a-0f46-4059-af58-50642f70e590" />
</p>

## 📜 License
MIT License
