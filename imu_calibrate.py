"""IMU calibration helper.

Usage:
  python imu_calibrate.py --accel-files plusX.csv minusX.csv plusY.csv minusY.csv plusZ.csv minusZ.csv \
      --gyro-files gyro1.csv gyro2.csv --out calib.json

The accel files must be six static recordings taken with the sensor held
so each axis sees +1g and -1g (order above). Gyro files should be stationary
recordings to estimate bias/noise.

The script writes a JSON file with accel bias/scale and gyro bias.
Save the JSON as `calib.json` and `data_parse_analysis.py` will auto-load it.
"""

import argparse
import json
import numpy as np
import pandas as pd
from datetime import datetime


def mean_accel_from_file(path):
    df = pd.read_csv(path)
    # expect columns ax,ay,az
    return np.array([df['ax'].mean(), df['ay'].mean(), df['az'].mean()], dtype=float)


def mean_gyro_from_files(paths):
    arrs = []
    for p in paths:
        df = pd.read_csv(p)
        if all(c in df.columns for c in ('gx','gy','gz')):
            arrs.append(df[['gx','gy','gz']].to_numpy())
    if len(arrs) == 0:
        return np.zeros(3), np.zeros(3)
    allg = np.vstack(arrs)
    return np.mean(allg, axis=0), np.std(allg, axis=0)


def extract_static_segments(file, out_dir='poses', min_seg_sec=2.0, std_window_sec=0.5, max_segments=6):
    """Detect stationary segments in a long log and save the longest `max_segments` as CSVs.

    Strategy:
    - compute accel magnitude
    - compute rolling std over `std_window_sec`
    - find regions where std < threshold (threshold = small value from median)
    - merge contiguous regions and pick longest `max_segments`
    """
    import os
    df = pd.read_csv(file)
    if 't_us' in df.columns:
        t = df['t_us'].to_numpy() * 1e-6
    elif 't_s' in df.columns:
        t = df['t_s'].to_numpy()
    else:
        fs = 30.0
        t = np.arange(len(df)) / fs

    dt = np.diff(t)
    fs = 1.0 / np.median(dt) if len(dt) > 0 else 30.0

    if not all(c in df.columns for c in ('ax','ay','az')):
        raise RuntimeError('Input file must contain ax,ay,az')

    mag = np.sqrt(df['ax'].to_numpy()**2 + df['ay'].to_numpy()**2 + df['az'].to_numpy()**2)

    win = max(1, int(round(std_window_sec * fs)))
    # compute rolling std (simple) using numpy convolution
    pad = np.ones(win)
    c1 = np.convolve(mag, pad, mode='same') / win
    c2 = np.convolve(mag*mag, pad, mode='same') / win
    roll_std = np.sqrt(np.maximum(0, c2 - c1*c1))

    median_std = np.median(roll_std)
    thr = max(1e-3, 3.0 * median_std)

    is_stat = roll_std <= thr

    # find contiguous true segments
    edges = np.diff(is_stat.astype(int))
    starts = np.where(edges == 1)[0] + 1
    ends = np.where(edges == -1)[0] + 1
    if is_stat[0]:
        starts = np.hstack(([0], starts))
    if is_stat[-1]:
        ends = np.hstack((ends, [len(is_stat)]))

    segs = []
    for s, e in zip(starts, ends):
        duration = t[e-1] - t[s]
        if duration >= min_seg_sec:
            segs.append((s, e, duration))

    if len(segs) == 0:
        raise RuntimeError('No stationary segments found; try lowering min_seg_sec or increasing recording length')

    # pick longest segments
    segs_sorted = sorted(segs, key=lambda x: x[2], reverse=True)[:max_segments]

    os.makedirs(out_dir, exist_ok=True)
    saved = []
    for i, (s, e, dur) in enumerate(segs_sorted, start=1):
        outp = os.path.join(out_dir, f'pose_{i}.csv')
        df.iloc[s:e].to_csv(outp, index=False)
        saved.append(outp)

    return saved


def compute_accel_cal(means_list):
    """Compute accel bias/scale from an UNORDERED list of six mean vectors.

    Algorithm:
    - For each mean vector, find which axis (x/y/z) has the largest absolute value.
    - Group means by axis and pair the two values for + and - signs (or by magnitude if signs same).
    - Compute bias = (mp + mn)/2 and scale = g / abs((mp - mn)/2).

    Returns (bias_list, scale_list)
    """
    g = 9.80665
    means = [np.asarray(m).reshape(3,) for m in means_list]
    # assign each mean to an axis by largest abs component
    axis_groups = {0: [], 1: [], 2: []}
    for m in means:
        idx = int(np.argmax(np.abs(m)))
        axis_groups[idx].append(m)

    bias = np.zeros(3)
    scale = np.ones(3)

    for axis in (0, 1, 2):
        grp = axis_groups[axis]
        if len(grp) < 2:
            # fallback: try to use overall means (not ideal)
            vals = [m[axis] for m in means]
            mp_val = max(vals)
            mn_val = min(vals)
        else:
            vals = [m[axis] for m in grp]
            # prefer pairing by opposite sign
            if any(v < 0 for v in vals) and any(v > 0 for v in vals):
                mp_val = max(vals)
                mn_val = min(vals)
            else:
                # no opposite signs found; pair largest and smallest
                mp_val = max(vals)
                mn_val = min(vals)

        bias[axis] = (mp_val + mn_val) / 2.0
        halfspan = (mp_val - mn_val) / 2.0
        halfspan_abs = abs(halfspan)
        if halfspan_abs < 1e-6:
            scale[axis] = 1.0
        else:
            scale[axis] = g / halfspan_abs

    return bias.tolist(), scale.tolist()


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--accel-files', nargs=6, help='Six accel files in order: +X -X +Y -Y +Z -Z')
    p.add_argument('--gyro-files', nargs='*', help='Stationary gyro files to estimate bias/std', default=[])
    p.add_argument('--out', default='calib.json')
    p.add_argument('--extract-from', help='Single long log CSV to auto-extract six pose segments')
    p.add_argument('--out-dir', help='Directory to save extracted pose CSVs', default='poses')
    p.add_argument('--min-seg-sec', type=float, default=2.0, help='Minimum stationary segment length (s)')
    args = p.parse_args()
    # If extract-from is provided, detect and save six longest stationary segments
    if args.extract_from:
        print(f'Extracting poses from {args.extract_from} ...')
        saved = extract_static_segments(args.extract_from, out_dir=args.out_dir, min_seg_sec=args.min_seg_sec)
        print('Saved pose files:')
        for s in saved:
            print('  ', s)
        # if we extracted exactly 6, set as accel-files for calibration
        if len(saved) >= 6:
            accel_files = saved[:6]
        else:
            raise RuntimeError('Extracted fewer than 6 pose segments; re-record with clearer stationary poses')
    else:
        accel_files = args.accel_files

    means = [mean_accel_from_file(f) for f in accel_files]
    accel_bias, accel_scale = compute_accel_cal(means)

    gyro_bias, gyro_std = mean_gyro_from_files(args.gyro_files)

    calib = {
        'accel_bias': accel_bias,
        'accel_scale': accel_scale,
        'gyro_bias': gyro_bias.tolist(),
        'gyro_std': gyro_std.tolist(),
        'created': datetime.utcnow().isoformat() + 'Z'
    }

    with open(args.out, 'w') as fh:
        json.dump(calib, fh, indent=2)

    print(f'Wrote calibration to {args.out}')
    print('Accel bias:', accel_bias)
    print('Accel scale:', accel_scale)
    print('Gyro bias:', gyro_bias)
    print('Gyro std:', gyro_std)


if __name__ == '__main__':
    main()
