import csv
import matplotlib.pyplot as plt

time = []
error = []
angle = []
prop = []
deriv = []

with open("wall_follow_log.csv") as f:
    reader = csv.DictReader(f)
    for row in reader:
        time.append(float(row["time"]))
        error.append(float(row["error"]))
        angle.append(float(row["angle"]))
        prop.append(float(row["proportional"]))
        deriv.append(float(row["derivative"]))
    idx = 0    
    for x in angle:
        angle[idx] = x * 10
        idx += 1



plt.plot(time, error, label="Error")
plt.plot(time, angle, label="Angle")
plt.plot(time, prop, label="Proportional")
plt.plot(time, deriv, label="Derivative")
plt.legend()
plt.grid()
plt.savefig("line_follow_plot.png", dpi=300, bbox_inches="tight")
