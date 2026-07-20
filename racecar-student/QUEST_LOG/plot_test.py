import csv
import matplotlib.pyplot as plt

time, error, angle, prop, deriv = [], [], [], [], []

with open("line_follow_log.csv") as f:
    reader = csv.DictReader(f)
    for row in reader:
        time.append(float(row["time"]))
        error.append(float(row["error"]))
        angle.append(float(row["angle"]))
        prop.append(float(row["proportional"]))
        deriv.append(float(row["derivative"]))

fig, ax_err = plt.subplots(figsize=(12, 6))

# Left axis: error in pixels (the big-swinging signal)
ax_err.plot(time, error, label="Error (px)", color="tab:blue", linewidth=1.2)
ax_err.axhline(0, color="gray", linestyle=":", linewidth=0.8)
ax_err.set_xlabel("time (s)")
ax_err.set_ylabel("Error (pixels)", color="tab:blue")
ax_err.tick_params(axis="y", labelcolor="tab:blue")
ax_err.grid(alpha=0.3)

# Right axis: the controller output, on its own +/-1 scale so it's actually visible
ax_ctl = ax_err.twinx()
ax_ctl.plot(time, angle, label="Angle (cmd)", color="tab:orange", linewidth=1.5)
ax_ctl.plot(time, prop,  label="P term", color="tab:green", linewidth=1.0, alpha=0.8)
ax_ctl.plot(time, deriv, label="D term", color="tab:red", linewidth=1.0, alpha=0.8)
ax_ctl.set_ylim(-1.2, 1.2)
ax_ctl.axhline(1,  color="red", linestyle="--", linewidth=0.6, alpha=0.5)
ax_ctl.axhline(-1, color="red", linestyle="--", linewidth=0.6, alpha=0.5)
ax_ctl.set_ylabel("Controller output (angle units)")

# Combine legends from both axes
lines = ax_err.get_lines()[:1] + ax_ctl.get_lines()[:3]
ax_err.legend(lines, [l.get_label() for l in lines], loc="lower left")

plt.title("Error vs. controller response (red dashes = steering saturation)")
plt.tight_layout()
plt.savefig("line_follow_plot.png", dpi=150, bbox_inches="tight")
print("saved line_follow_plot.png")