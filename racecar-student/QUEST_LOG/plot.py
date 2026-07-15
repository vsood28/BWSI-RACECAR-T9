import csv
import matplotlib.pyplot as plt

time = []
error = []
angle = []

with open("wall_follow_log.csv") as f:
    reader = csv.DictReader(f)
    for row in reader:
        time.append(float(row["time"]))
        error.append(float(row["error"]))
        angle.append(float(row["angle"]))

plt.plot(time, error, label="Error")
plt.plot(time, angle, label="Angle")
plt.legend()
plt.grid()
plt.show()