"""Clean accelerometer -> position pipeline and plots.

Reads the first CSV matching "log_*.csv" (expects `t_us`, `ax`, `ay`, `az` columns).
Performs bias removal, low-pass filtering, integrates to velocity and position,
applies detrending to reduce drift, and plots three acceleration vs time panels
and a 3D position trajectory.
"""

import glob
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from scipy.signal import butter, filtfilt, detrend

import math


class MadgwickAHRS:
	"""A compact Madgwick AHRS implementation (IMU-only update used here).

	Methods
	- update_imu(gyr, acc): feed gyro (rad/s) and accel (m/s^2 or normalized) per sample
	  returns quaternion [w,x,y,z]
	"""

	def __init__(self, sample_period=1 / 256.0, beta=0.1):
		self.sample_period = float(sample_period)
		self.beta = float(beta)
		self.quaternion = np.array([1.0, 0.0, 0.0, 0.0], dtype=float)

	def update_imu(self, gyr, acc):
		q1, q2, q3, q4 = self.quaternion
		gx, gy, gz = gyr
		ax, ay, az = acc

		# normalize accelerometer
		norm = math.sqrt(ax * ax + ay * ay + az * az)
		if norm == 0:
			return self.quaternion
		ax /= norm; ay /= norm; az /= norm

		# Auxiliary variables to avoid repeated arithmetic
		_2q1 = 2.0 * q1
		_2q2 = 2.0 * q2
		_2q3 = 2.0 * q3
		_2q4 = 2.0 * q4

		# Objective function and Jacobian (IMU)
		f1 = _2q2 * q4 - _2q1 * q3 - ax
		f2 = _2q1 * q2 + _2q3 * q4 - ay
		f3 = 1.0 - _2q2 * q2 - _2q3 * q3 - az

		J_11or24 = _2q3
		J_12or23 = _2q4
		J_13or22 = _2q1
		J_14or21 = _2q2

		grad1 = J_14or21 * f2 - J_11or24 * f1
		grad2 = J_12or23 * f1 + J_13or22 * f2
		grad3 = J_12or23 * f2 - J_13or22 * f1
		grad4 = J_14or21 * f1 + J_11or24 * f2

		grad = np.array([grad1, grad2, grad3, grad4], dtype=float)
		gnorm = np.linalg.norm(grad)
		if gnorm > 0:
			grad /= gnorm

		# q_dot = 0.5 * q * omega - beta * grad
		omega = np.hstack(([0.0], np.array([gx, gy, gz], dtype=float)))
		qdot = 0.5 * self._quaternion_product(self.quaternion, omega) - self.beta * grad

		self.quaternion += qdot * self.sample_period
		self.quaternion /= np.linalg.norm(self.quaternion)
		return self.quaternion

	def update(self, gyr, acc, mag):
		"""Madgwick update using magnetometer for heading stabilization.

		gyr: [gx,gy,gz] in rad/s
		acc: [ax,ay,az] in m/s^2 (or normalized)
		mag: [mx,my,mz] (any units)
		"""
		q1, q2, q3, q4 = self.quaternion
		gx, gy, gz = gyr
		ax, ay, az = acc
		mx, my, mz = mag

		# normalize accelerometer
		norm_a = math.sqrt(ax * ax + ay * ay + az * az)
		if norm_a == 0:
			return self.quaternion
		ax /= norm_a; ay /= norm_a; az /= norm_a

		# normalize magnetometer
		norm_m = math.sqrt(mx * mx + my * my + mz * mz)
		if norm_m == 0:
			return self.update_imu(gyr, acc)
		mx /= norm_m; my /= norm_m; mz /= norm_m

		# Reference direction of Earth's magnetic field
		# compute auxiliary variables
		_2q1 = 2.0 * q1
		_2q2 = 2.0 * q2
		_2q3 = 2.0 * q3
		_2q4 = 2.0 * q4
		_4q1 = 4.0 * q1
		_4q2 = 4.0 * q2
		_4q3 = 4.0 * q3
		_8q2 = 8.0 * q2
		_8q3 = 8.0 * q3
		q1q1 = q1 * q1
		q2q2 = q2 * q2
		q3q3 = q3 * q3
		q4q4 = q4 * q4

		# compute reference direction of magnetic field
		hx = 2.0 * (mx * (0.5 - q3q3 - q4q4) + my * (q2 * q3 - q1 * q4) + mz * (q2 * q4 + q1 * q3))
		hy = 2.0 * (mx * (q2 * q3 + q1 * q4) + my * (0.5 - q2q2 - q4q4) + mz * (q3 * q4 - q1 * q2))
		_2bx = math.sqrt(hx * hx + hy * hy)
		_2bz = 2.0 * (mx * (q2 * q4 - q1 * q3) + my * (q1 * q2 + q3 * q4) + mz * (0.5 - q2q2 - q3q3))

		# Gradient descent algorithm corrective step (with mag)
		f1 = _2q2 * q4 - _2q1 * q3 - ax
		f2 = _2q1 * q2 + _2q3 * q4 - ay
		f3 = 1.0 - _2q2 * q2 - _2q3 * q3 - az
		f4 = _2bx * (0.5 - q3q3 - q4q4) + _2bz * (q2 * q4 - q1 * q3) - mx
		f5 = _2bx * (q2 * q3 - q1 * q4) + _2bz * (q1 * q2 + q3 * q4) - my

		# Jacobian elements (approx)
		J_11 = -_2q3
		J_12 = _2q4
		J_13 = -_2q1
		J_14 = _2q2

		# build gradient (simplified combined)
		grad = np.zeros(4, dtype=float)
		grad[0] = J_11 * f1 + J_12 * f2 + J_13 * f3
		grad[1] = J_14 * f1 - J_11 * f2 + J_12 * f3
		grad[2] = J_13 * f1 + J_14 * f2 - J_11 * f3
		grad[3] = J_12 * f1 - J_13 * f2 + J_14 * f3

		gnorm = np.linalg.norm(grad)
		if gnorm > 0:
			grad /= gnorm

		omega = np.hstack(([0.0], np.array([gx, gy, gz], dtype=float)))
		qdot = 0.5 * self._quaternion_product(self.quaternion, omega) - self.beta * grad

		self.quaternion += qdot * self.sample_period
		self.quaternion /= np.linalg.norm(self.quaternion)
		return self.quaternion

	@staticmethod
	def _quaternion_product(q, r):
		w1, x1, y1, z1 = q
		w2, x2, y2, z2 = r
		return np.array([
			w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
			w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
			w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
			w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
		], dtype=float)


def quat_to_rotmat(q):
	w, x, y, z = q
	R = np.array([
		[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
		[2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
		[2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
	], dtype=float)
	return R


def lowpass(data, fs, fc, order=3):
	ny = 0.5 * fs
	w = min(max(fc / ny, 1e-6), 0.999)
	b, a = butter(order, w, btype="low")
	return filtfilt(b, a, data)


def integrate_trap(a, t):
    dt = np.diff(t)
    out = np.zeros_like(t)
    if len(t) < 2:
        return out
    # trapezoidal integration
    out[1:] = np.cumsum((a[:-1] + a[1:]) / 2.0 * dt)
    return out


def process_file(path, n_bias=200, fc_acc=5.0):
	df = pd.read_csv(path)

	# apply calibration if present
	try:
		import json
		with open('calib.json', 'r') as fh:
			calib = json.load(fh)
		ab = np.array(calib.get('accel_bias', [0,0,0]), dtype=float)
		ascale = np.array(calib.get('accel_scale', [1,1,1]), dtype=float)
		gb = np.array(calib.get('gyro_bias', [0,0,0]), dtype=float)
		if 'ax' in df.columns:
			df['ax'] = (df['ax'] - ab[0]) * ascale[0]
			df['ay'] = (df['ay'] - ab[1]) * ascale[1]
			df['az'] = (df['az'] - ab[2]) * ascale[2]
		if 'gx' in df.columns:
			df['gx'] = df['gx'] - gb[0]
			df['gy'] = df['gy'] - gb[1]
			df['gz'] = df['gz'] - gb[2]
		print('Loaded calibration from calib.json and applied to dataframe')
	except FileNotFoundError:
		pass
	except Exception as e:
		print('Warning: failed to load/apply calib.json:', e)

	if "t_us" in df.columns:
		t = df["t_us"].to_numpy() * 1e-6
	elif "t_s" in df.columns:
		t = df["t_s"].to_numpy()
	else:
		# fallback to index with assumed sample rate
		fs_guess = 30.0
		t = np.arange(len(df)) / fs_guess

	# sample rate estimate
	dt = np.diff(t)
	if len(dt) == 0:
		raise RuntimeError("Not enough samples in file")
	fs = 1.0 / np.median(dt)

	# read accel columns (expecting these names)
	ax = df["ax"].to_numpy()
	ay = df["ay"].to_numpy()
	az = df["az"].to_numpy()

	n_bias = int(min(len(df), n_bias))
	ax0 = ax - np.mean(ax[:n_bias])
	ay0 = ay - np.mean(ay[:n_bias])
	az0 = az - np.mean(az[:n_bias])

	# read gyro (rad/s) and magnetometer if available
	if all(c in df.columns for c in ("gx", "gy", "gz")):
		gx = df["gx"].to_numpy()
		gy = df["gy"].to_numpy()
		gz = df["gz"].to_numpy()
	else:
		# fallbacks
		gx = df["gx"].to_numpy() if "gx" in df.columns else np.zeros_like(ax)
		gy = df["gy"].to_numpy() if "gy" in df.columns else np.zeros_like(ay)
		gz = df["gz"].to_numpy() if "gz" in df.columns else np.zeros_like(az)

	# guess units: if median magnitude large -> degrees/sec, convert to rad/s
	med_g = np.median(np.abs(np.hstack((gx, gy, gz))))
	if med_g > 50:  # likely deg/s
		gx = np.deg2rad(gx)
		gy = np.deg2rad(gy)
		gz = np.deg2rad(gz)

	# magnetometer (optional)
	if all(c in df.columns for c in ("mx", "my", "mz")):
		mx = df["mx"].to_numpy()
		my = df["my"].to_numpy()
		mz = df["mz"].to_numpy()
		# Magnetometer outlier rejection: detect spikes and magnitude outliers
		mag_mag = np.sqrt(mx * mx + my * my + mz * mz)
		median_mag = np.median(mag_mag)
		mad = np.median(np.abs(mag_mag - median_mag))
		# threshold: either percent of median or a few MADs
		thr_mag = max(0.3 * median_mag, 3.0 * mad, 1e-6)
		# jump threshold between consecutive samples
		jump_thr = max(0.3 * median_mag, 0.5)
		valid = np.abs(mag_mag - median_mag) <= thr_mag
		# remove sudden large jumps
		diff = np.hstack(([0.0], np.abs(np.diff(mag_mag))))
		valid &= diff <= jump_thr
		# if too many bad samples, disable mag fusion
		if valid.sum() < max(3, int(0.05 * len(valid))):
			mag_available = False
		else:
			mag_available = True
			# forward-fill bad samples using last good value, fallback to median of good samples
			mx_clean = mx.copy()
			my_clean = my.copy()
			mz_clean = mz.copy()
			mx_med = np.median(mx[valid])
			my_med = np.median(my[valid])
			mz_med = np.median(mz[valid])
			last_good = None
			for i in range(len(valid)):
				if valid[i]:
					last_good = i
				else:
					if last_good is not None:
						mx_clean[i] = mx_clean[last_good]
						my_clean[i] = my_clean[last_good]
						mz_clean[i] = mz_clean[last_good]
					else:
						mx_clean[i] = mx_med
						my_clean[i] = my_med
						mz_clean[i] = mz_med
			mx = mx_clean
			my = my_clean
			mz = mz_clean
	else:
		mx = my = mz = None
		mag_available = False

	# run Madgwick AHRS (IMU-only if mag not present)
	madgwick = MadgwickAHRS(sample_period=1.0 / fs, beta=0.08)
	quats = np.zeros((len(t), 4), dtype=float)
	for i in range(len(t)):
		gyr = np.array([gx[i], gy[i], gz[i]], dtype=float)
		acc = np.array([ax0[i], ay0[i], az0[i]], dtype=float)
		if mag_available:
			mag = np.array([mx[i], my[i], mz[i]], dtype=float)
			q = madgwick.update(gyr, acc, mag)
		else:
			q = madgwick.update_imu(gyr, acc)
		quats[i, :] = q

	# rotate body accel -> earth frame and subtract gravity
	acc_earth = np.zeros((len(t), 3), dtype=float)
	for i in range(len(t)):
		R = quat_to_rotmat(quats[i, :])
		# rotate body to earth: a_e = R @ a_b
		acc_earth[i, :] = R.dot(np.array([ax[i], ay[i], az[i]], dtype=float))
	# subtract gravity (z-up)
	g_vec = np.array([0.0, 0.0, 9.80665])
	acc_lin_earth = acc_earth - g_vec[np.newaxis, :]

	# filter in Earth frame
	ax_f = lowpass(acc_lin_earth[:, 0], fs, fc_acc)
	ay_f = lowpass(acc_lin_earth[:, 1], fs, fc_acc)
	az_f = lowpass(acc_lin_earth[:, 2], fs, fc_acc)

	vx = integrate_trap(ax_f, t)
	vy = integrate_trap(ay_f, t)
	vz = integrate_trap(az_f, t)

	# detrend velocity to reduce low-frequency drift (linear component)
	vx = detrend(vx, type="linear")
	vy = detrend(vy, type="linear")
	vz = detrend(vz, type="linear")

	px = integrate_trap(vx, t)
	py = integrate_trap(vy, t)
	pz = integrate_trap(vz, t)

	# detect start/stop of motion using filtered accel magnitude and bias noise
	accel_mag = np.sqrt(ax_f * ax_f + ay_f * ay_f + az_f * az_f)
	bias_mag = np.sqrt((ax0[:n_bias] ** 2) + (ay0[:n_bias] ** 2) + (az0[:n_bias] ** 2))
	a_std_bias = np.std(bias_mag) if len(bias_mag) > 0 else 0.0
	a_thresh = max(0.05, 5.0 * a_std_bias)
	moving = accel_mag > a_thresh
	inds = np.where(moving)[0]
	if inds.size > 0:
		start_idx = int(inds[0])
		stop_idx = int(inds[-1])
	else:
		start_idx = 0
		stop_idx = len(t) - 1

	return {
		"t": t,
		"ax_raw": ax,
		"ay_raw": ay,
		"az_raw": az,
		"ax_f": ax_f,
		"ay_f": ay_f,
		"az_f": az_f,
		"vx": vx,
		"vy": vy,
		"vz": vz,
		"px": px,
		"py": py,
		"pz": pz,
		"fs": fs,
		"start_idx": start_idx,
		"stop_idx": stop_idx,
	}


def plot_results(res):
	t = res["t"]
	fig, axs = plt.subplots(3, 1, sharex=True, figsize=(10, 8))
	axs[0].plot(t, res["ax_raw"], label="raw", alpha=0.6)
	axs[0].plot(t, res["ax_f"], label="filtered", linewidth=1.2)
	axs[0].set_ylabel("ax [m/s^2]")
	axs[0].legend()
	axs[0].grid(True)

	axs[1].plot(t, res["ay_raw"], label="raw", alpha=0.6)
	axs[1].plot(t, res["ay_f"], label="filtered", linewidth=1.2)
	axs[1].set_ylabel("ay [m/s^2]")
	axs[1].legend()
	axs[1].grid(True)

	axs[2].plot(t, res["az_raw"], label="raw", alpha=0.6)
	axs[2].plot(t, res["az_f"], label="filtered", linewidth=1.2)
	axs[2].set_ylabel("az [m/s^2]")
	axs[2].set_xlabel("time [s]")
	axs[2].legend()
	axs[2].grid(True)

	# add start/stop vertical markers
	start_idx = res.get("start_idx", 0)
	stop_idx = res.get("stop_idx", len(t) - 1)
	t_start = t[start_idx]
	t_stop = t[stop_idx]
	for ax in axs:
		ax.axvline(t_start, color="green", linestyle="--", linewidth=1.2)
		ax.axvline(t_stop, color="red", linestyle="--", linewidth=1.2)
		ylim = ax.get_ylim()
		ax.text(t_start, ylim[1] - 0.05 * (ylim[1] - ylim[0]), "start", color="green", ha="right")
		ax.text(t_stop, ylim[1] - 0.05 * (ylim[1] - ylim[0]), "stop", color="red", ha="left")

	plt.tight_layout()

	# 3D position
	fig = plt.figure(figsize=(10, 6))
	ax3 = fig.add_subplot(111, projection="3d")
	ax3.plot(res["px"], res["py"], res["pz"], linewidth=2)
	ax3.set_xlabel("X [m]")
	ax3.set_ylabel("Y [m]")
	ax3.set_zlabel("Z [m]")
	ax3.set_title("3D Position (integrated from accel)")
	ax3.grid(True)

	# mark start and stop positions in 3D
	ps = res
	s_idx = res.get("start_idx", 0)
	e_idx = res.get("stop_idx", len(t) - 1)
	ax3.scatter(ps["px"][s_idx], ps["py"][s_idx], ps["pz"][s_idx], color="green", s=60, label="start")
	ax3.scatter(ps["px"][e_idx], ps["py"][e_idx], ps["pz"][e_idx], color="red", s=60, label="stop")
	ax3.legend()

	plt.tight_layout()
	plt.show()


def main():
	files = glob.glob("log_*.csv")
	if len(files) == 0:
		print("No log_*.csv files found in current directory")
		return
	path = files[0]
	print(f"Processing: {path}")
	res = process_file(path)
	print(f"Estimated sample rate: {res['fs']:.2f} Hz")
	plot_results(res)


if __name__ == "__main__":
	main()