import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from os import listdir
from os.path import isfile, join
from scipy.spatial import ConvexHull
from scipy.spatial.transform import Rotation as R


class Cuboid:
    def __init__(self, position, size, orientation):
        self.x = position[0]
        self.y = position[1]
        self.z = position[2]
        self.l = size[0]
        self.w = size[1]
        self.h = size[2]
        self.pts = np.array(
            [
                [-self.l / 2, -self.w / 2, -self.h / 2],
                [-self.l / 2, -self.w / 2, +self.h / 2],
                [-self.l / 2, +self.w / 2, -self.h / 2],
                [-self.l / 2, +self.w / 2, +self.h / 2],
                [+self.l / 2, -self.w / 2, -self.h / 2],
                [+self.l / 2, -self.w / 2, +self.h / 2],
                [+self.l / 2, +self.w / 2, -self.h / 2],
                [+self.l / 2, +self.w / 2, +self.h / 2],
            ]
        )
        self.trnsfm_pts = []
        self.transform(position, orientation)
        self.hull = ConvexHull(self.pts)

    def transform(self, pt, q):
        r = R.from_quat(q)
        self.trnsfm_pts = r.apply(self.pts) + pt

    def plot(self, ax, c):
        ax.plot_trisurf(
            self.trnsfm_pts[:, 0],
            self.trnsfm_pts[:, 1],
            self.trnsfm_pts[:, 2],
            triangles=self.hull.simplices,
            shade=False,
            edgecolor="k",
            color=c,
        )


class SolutionAnimator:
    def __init__(self, obs, robot, fname, infolder, outfolder):
        self.out_folder = outfolder
        self.fname = fname
        self.obs = obs
        self.robot = robot
        data = np.genfromtxt(infolder + fname, delimiter=" ")
        self.pos = data[:, 0:3]
        self.rot = data[:, 3:8]

    def get_path_info(self, idx):
        return self.pos[idx, :], self.rot[idx, :]

    def SolutionAnimate(self):
        fig, (ax1, ax2) = plt.subplots(1, 2)

    def AzimuthAnimate(self):
        fig = plt.figure()
        ax = plt.axes(projection="3d")
        self.obs.plot(ax, "red")
        ax.plot3D(
            self.pos[:, 0], self.pos[:, 1], self.pos[:, 2], color="black",
        )
        ax.set_xlim([0, 15])
        ax.set_ylim([-10, 10])
        ax.set_zlim([-10, 10])

        def animate(i):
            # azimuth angle : 0 deg to 360 deg
            ax.view_init(elev=10, azim=i * 1)
            return (fig,)

        ani = FuncAnimation(fig, animate, frames=600, interval=20, blit=True)
        ani.save(
            self.out_folder + self.fname.split(".")[0] + ".gif",
            writer="pillow",
            fps=1000 / 20,
        )


obs = Cuboid(np.array([5, 0, 0]), np.array([5, 5, 5]), np.array([0, 0, 0, 1]))
robot = Cuboid(np.array([0, 0, 0]), np.array([2, 1, 1]), np.array([0, 0, 0, 1]))

files = [f for f in listdir("solutions") if isfile(join("solutions", f))]
for fname in files:
    print(f"Working on {fname}...")
    SA = SolutionAnimator(obs, robot, fname, "solutions/", "visualization/")
    SA.AzimuthAnimate()
