import pandas as pd
import numpy as np
import glob
import matplotlib.pyplot as plt
from scipy.signal import butter, filtfilt


# Find the first log file matching the pattern
log_file = glob.glob("log_*.csv")[0]
df = pd.read_csv(log_file)

# time in seconds
df["t_s"] = df["t_us"] * 1e-6
t = df["t_s"].to_numpy()

alt = df["altM"].to_numpy()
az  = df["az"].to_numpy()
gz  = df["gz"].to_numpy()

# remove gravity / bias using first 200 samples (adjust as needed)
n_bias = min((200), len(df))
az_lin = az - np.mean(az[:n_bias])


# --- Low-pass filter ---
fs = 30.0              # your sample rate Hz
fc = 3.0               # cutoff freq Hz (tune 3‑10)
b, a = butter(2, fc/(fs/2), btype='low')
az_filt = filtfilt(b, a, az_lin)

fs = 30.0              # your sample rate Hz
fc = 1.0               # cutoff freq Hz (tune 3‑10)
b, a = butter(2, fc/(fs/2), btype='low')
alt_filt = filtfilt(b, a, alt)


# --- Velocity estimates ---

dt = np.diff(t)
az_mid = (az_lin[:-1] + az_lin[1:]) / 2.0
az_mid_filt = (az_filt[:-1] + az_filt[1:]) / 2.0


v_from_a = np.zeros_like(t)
v_from_a[1:] = np.cumsum(az_mid * dt)
v_from_a_filt = np.zeros_like(t)
v_from_a_filt[1:] = np.cumsum(az_mid_filt * dt)

v_from_alt = np.zeros_like(t)
v_from_alt[1:] = np.diff(alt) / dt
v_from_alt_filt = np.zeros_like(t)
v_from_alt_filt[1:] = np.diff(alt_filt) / dt

alpha = 0.1  # small correction from alt
v_blend = (1-alpha)*v_from_a + alpha*v_from_alt

print(v_from_a_filt)
print(v_from_a)

# --- Apogee ---

apogee_idx = np.argmax(alt)
apogee_alt = alt[apogee_idx]
apogee_time = t[apogee_idx]
print(f"Apogee ~ {apogee_alt:.1f} m at t = {apogee_time:.2f} s")

# --- Plots ---

fig, axs = plt.subplots(4, 1, sharex=True, figsize=(10, 10))

axs[0].plot(t, alt)
axs[0].set_ylabel("Alt [m]")
axs[0].grid(True)

axs[1].plot(t, az_lin)
axs[1].set_ylabel("a_z lin [m/s²]")
axs[1].grid(True)

axs[2].plot(t, v_from_a, label="v from accel")
axs[2].plot(t, v_from_alt, label="v from alt", alpha=0.7)
axs[2].set_ylabel("v [m/s]")
axs[2].legend()
axs[2].grid(True)

axs[3].plot(t, gz)
axs[3].set_ylabel("gz [rad/s]")
axs[3].set_xlabel("time [s]")
axs[3].grid(True)

fig, ax = plt.subplots(figsize=(10, 6))
ax.plot(t, v_from_a, label="v from accel (unfiltered)", linewidth=1.5)
ax.plot(t, v_from_a_filt, label="v from accel (filtered)", linewidth=1.5)
ax.set_xlabel("time [s]")
ax.set_ylabel("v [m/s]")
ax.set_title("Velocity Comparison: Filtered vs Unfiltered")
ax.legend()
ax.grid(True)

fig, ax = plt.subplots(figsize=(10, 6))
ax.plot(t, v_from_alt, label="v from alt (unfiltered)", linewidth=1.5)
ax.plot(t, v_from_alt_filt, label="v from alt (filtered)", linewidth=1.5)
ax.set_xlabel("time [s]")
ax.set_ylabel("v [m/s]")
ax.set_title("Velocity Comparison: Filtered vs Unfiltered (from Altitude)")
ax.legend()
ax.grid(True)


fig, ax = plt.subplots(figsize=(10, 6))
ax.plot(t, v_from_a, label="v from accel", linewidth=1.5)
ax.plot(t, v_from_alt, label="v from alt", linewidth=1.5)
ax.plot(t, v_blend, label="v blended", linewidth=1.5)
ax.set_xlabel("time [s]")
ax.set_ylabel("v [m/s]")
ax.set_title("Velocity Comparison: All Methods")
ax.legend()
ax.grid(True)


plt.tight_layout()
plt.show()

