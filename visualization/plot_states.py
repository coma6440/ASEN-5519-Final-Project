import numpy as np
import matplotlib.pyplot as plt
from os import listdir
from os.path import isfile, join, exists

geo_files = [
    f
    for f in listdir("solutions/geometric/")
    if isfile(join("solutions/geometric/", f))
]

kino_files = [
    f
    for f in listdir("solutions/kinodynamic/")
    if isfile(join("solutions/kinodynamic/", f))
]

state_labels = ["X", "Y", "Z", "q_x", "q_y", "q_z", "q_w", "v"]

for fname in geo_files:
    f = "solutions/geometric/" + fname
    data = np.asarray(np.genfromtxt(f, delimiter=" "))
    a = np.vstack([data[i] for i in range(len(data))])
    (r, c) = np.shape(a)
    fig, axs = plt.subplots(8)
    for i in range(8):
        axs[i].plot(range(r), data[:, i])
        axs[i].set_ylabel(state_labels[i])
    plt.savefig("visualization/" + fname.split(".")[0] + ".png")

for fname in kino_files:
    f = "solutions/kinodynamic/" + fname
    data = np.asarray(np.genfromtxt(f, delimiter=" "))
    a = np.vstack([data[i] for i in range(len(data))])
    (r, c) = np.shape(a)
    fig, axs = plt.subplots(8, sharex=True)
    for i in range(1, r):
        data[i, -1] += data[i - 1, -1]
    for i in range(8):
        axs[i].plot(data[:, -1], data[:, i])
        axs[i].set_ylabel(state_labels[i])
    plt.savefig("visualization/" + fname.split(".")[0] + ".png")
