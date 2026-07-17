import csv
import matplotlib.pyplot as plt

time = []
error = []
angle = []

with open("line_follow_log.csv") as f:
    reader = csv.DictReader(f)
    for row in reader:
        time.append(float(row["time"]))
        error.append(float(row["error"]))
        angle.append(float(row["angle"]))
    idx = 0    
    for x in angle:
        angle[idx] = x * 10
        idx += 1



plt.plot(time, error, label="Error")
plt.plot(time, angle, label="Angle")
plt.legend()
plt.grid()
plt.savefig("line_follow_plot.png", dpi=300, bbox_inches="tight")
