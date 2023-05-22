import matplotlib.pyplot as plt
import numpy as np

best = "20"
worse = "10"
selected = worse

x = []
y = []
z = []
count = 1
lines = open(f"{selected}.txt").readlines()
for t in lines:
    if count == 10:
        line = t.strip('()\n').split(',')
        end_x = float(line[1])
        end_y = float(line[2])
        end_z = float(line[3])

    if 11 <= count <= (len(lines) - 3):
        line = t.strip('()\n').split(',')
        x.append(float(line[1]))
        y.append(float(line[2]))
        z.append(float(line[3]))
    count = count + 1



xpoints = np.array(x)
ypoints = np.array(y)
zpoints = np.array(z)

plt.rcParams['font.size'] = 16
fig = plt.gcf()
fig.set_size_inches(8.5, 10)

plt.plot(xpoints, ypoints, label='Landing route')
plt.scatter(xpoints[0], ypoints[0], label='Start point')
plt.scatter(end_x, end_y, label='Desired landing point')
plt.xlabel('X (cm)')
plt.ylabel('Y (cm)')
plt.legend(loc='center left')
plt.grid()
plt.show()
fig.savefig(f"{selected}_xy.png", dpi=300)
plt.clf()

fig = plt.gcf()
fig.set_size_inches(8.5, 6)
end_z_line = np.full(len(ypoints), end_z)
plt.plot(ypoints, zpoints, label='Height route')
plt.plot(ypoints, end_z_line, label='Desired height')
plt.scatter(ypoints[-1], end_z_line[-1], label='Desired landing point', color='orange')
plt.xlabel('Y (cm)')
plt.ylabel('Z (cm)')
plt.legend()
plt.grid()
plt.show()
fig.savefig(f"{selected}_yz.png", dpi=300)