import numpy as np
import matplotlib.pyplot as plt
import control as ctrl

# =========================================================
# Discretization of First-Order Identified Model (Non-Parametric)
# =========================================================
Ts = 0.1
gp  = ctrl.tf([20], [50, 1])              # First-order plant
gpd = ctrl.c2d(gp, Ts, method='zoh')      # Discrete plant (ZOH)

# =========================================================
# PI controller parameters
# =========================================================
Kp = 0.8   # proportional gain
Ti = 9.0   # integral time

# PI discretization (incremental form)
K0 = Kp + Kp * Ts / (2.0 * Ti)
K1 = -Kp + Kp * Ts / (2.0 * Ti)

# =========================================================
# Extract discrete TF coefficients robustly
# Want MATLAB-like: y[k] = num(2)*u1 - den(2)*y1
# =========================================================
num, den = ctrl.tfdata(gpd)   # returns nested lists/arrays
num = np.asarray(num, dtype=float).squeeze()
den = np.asarray(den, dtype=float).squeeze()

# Ensure 1D
num = np.ravel(num)
den = np.ravel(den)

# Normalize so den[0] = 1
if den[0] != 1.0:
    num = num / den[0]
    den = den / den[0]

# If numerator comes as [b1] (size 1), treat it as [0, b1] so num(2)=b1
if num.size == 1:
    num = np.array([0.0, float(num[0])], dtype=float)

# Safety check (first-order discrete should be den size 2)
if den.size != 2:
    raise ValueError(f"Expected first-order discrete denominator of length 2, got {den.size}: {den}")

# MATLAB mapping: num(2) -> num[1], den(2) -> den[1]
b1 = float(num[1])
a1 = float(den[1])

# (Opcional) para verificar qué te devolvió control:
print("gpd =", gpd)
print("num =", num, "den =", den)
print("Using: y[k] = b1*u[k-1] - a1*y[k-1] with b1 =", b1, "a1 =", a1)

# =========================================================
# Closed-loop simulation (Direct difference equation)
# =========================================================
t   = np.arange(0, 60 + Ts, Ts)
Ref = np.ones(len(t))

y = np.zeros(len(t))
Usim = np.zeros(len(t))

# Histories
y1 = 0.0
u1 = 0.0
error1 = 0.0

for k in range(len(t)):
    # ---- PLANT ----
    y[k] = b1 * u1 - a1 * y1

    # ---- FEEDBACK ----
    error = Ref[k] - y[k]
    u = u1 + K0 * error + K1 * error1

    # ---- SATURATION ----
    if u > 100:
        u = 100
    if u < 0:
        u = 0

    # ---- UPDATE STATES ----
    y1 = y[k]
    u1 = u
    error1 = error
    Usim[k] = u

# =========================================================
# PLOTS
# =========================================================
plt.figure(figsize=(8, 6))

plt.subplot(2, 1, 1)
plt.plot(t, y, '+', markersize=4, label='Simulation')
plt.plot(t, Ref, '--', label='Reference')
plt.xlabel('Time [s]')
plt.ylabel('Response')
plt.legend()
plt.grid(True)

plt.subplot(2, 1, 2)
plt.plot(t, Usim, '+', markersize=4)
plt.xlabel('Time [s]')
plt.ylabel('Control signal')
plt.grid(True)

plt.tight_layout()
plt.show()
