import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

smooth_window = 5  # rolling mean window size

# ===== Load sensor data =====
df_sensor = pd.read_csv(r"C:\Users\cwruc\Documents\sand_sim\RFT CHARAC\bota_sensor_log.csv")
df_depth = pd.read_csv(
    r"C:\Users\cwruc\Documents\sand_sim\RFT CHARAC\depth.txt",
    skiprows=2,
    sep=",",
    usecols=[0, 1],
    header=None,
    names=["t", "y"],
).apply(pd.to_numeric, errors="coerce").dropna()

# ===== Make relative times =====
df_sensor = df_sensor.sort_values("sys_time").reset_index(drop=True)
df_sensor["t_rel"] = df_sensor["sys_time"] - df_sensor["sys_time"].iloc[0]

df_depth = df_depth.sort_values("t").reset_index(drop=True)
df_depth["t_rel"] = df_depth["t"] - df_depth["t"].iloc[0]

# ===== Merge by nearest time =====
paired = pd.merge_asof(
    df_sensor[["t_rel", "fz"]].sort_values("t_rel"),
    df_depth[["t_rel", "y"]].sort_values("t_rel"),
    on="t_rel",
    direction="nearest"
).dropna().rename(columns={"y": "depth"})

# ===== Apply light smoothing =====
paired["fz"] = paired["fz"].rolling(window=smooth_window, center=True, min_periods=1).mean()
paired["depth"] = paired["depth"].rolling(window=smooth_window, center=True, min_periods=1).mean()

# ===== Trim only Fz =====
t_start = 4.7
t_end = 6.22
fz_trimmed = paired.loc[(paired["t_rel"] >= t_start) & (paired["t_rel"] <= t_end), "fz"].reset_index(drop=True)
depth_for_plot = paired["depth"].iloc[:len(fz_trimmed)].reset_index(drop=True)

# ===== Fit a linear slope =====
coeffs = np.polyfit(depth_for_plot, fz_trimmed, 1)
slope, intercept = coeffs
fz_fit = slope * depth_for_plot + intercept

# ===== Plot Fz vs Depth with slope =====
plt.figure(figsize=(8, 6))
plt.plot(depth_for_plot, fz_trimmed, label="Data")
plt.plot(depth_for_plot, fz_fit, "--", color="red", label=f"Slope = {slope:.2f} N/unit depth")
plt.xlabel("Depth")
plt.ylabel("Fz (N)")
plt.title(f"Fz (trimmed {t_start}-{t_end}s) vs Depth with Linear Fit")
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.show()
