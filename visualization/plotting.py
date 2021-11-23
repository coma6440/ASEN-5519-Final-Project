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
        return self.trnsfm_pts

    def plot(self, ax, c, alpha=1):
        ob = ax.plot_trisurf(
            self.trnsfm_pts[:, 0],
            self.trnsfm_pts[:, 1],
            self.trnsfm_pts[:, 2],
            triangles=self.hull.simplices,
            shade=False,
            edgecolor="k",
            color=c,
            alpha=alpha,
        )
        return ob


class SolutionAnimator:
    def __init__(self, obs, robot, fname, infolder, outfolder):
        self.out_folder = outfolder
        self.fname = fname
        self.obs = obs
        self.robot = robot
        data = np.genfromtxt(infolder + fname, delimiter=" ")
        self.n = len(data)
        self.pos = data[:, 0:3]
        self.rot = data[:, 3:8]

    def get_path_info(self, idx):
        return self.pos[idx, :], self.rot[idx, :]

    def AzimuthAnimate(self):
        fig = plt.figure()
        ax = plt.axes(projection="3d")
        self.obs.plot(ax, "red")
        ax.plot3D(
            self.pos[:, 0],
            self.pos[:, 1],
            self.pos[:, 2],
            color="black",
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

    def SubplotAnimate(self):
        # Initial robot configuration
        p0, r0 = self.get_path_info(0)
        self.robot.transform(p0, r0)

        # Create the plots
        fig, (ax1, ax2) = plt.subplots(1, 2, subplot_kw={"projection": "3d"})

        # Add obstacles
        self.obs.plot(ax1, "red", 0.3)
        self.obs.plot(ax2, "red", 0.3)

        # Limits
        ax1.set_xlim([0, 15])
        ax1.set_ylim([-10, 10])
        ax1.set_zlim([-10, 10])
        ax2.set_xlim([0, 15])
        ax2.set_ylim([-10, 10])
        ax2.set_zlim([-10, 10])

        # Labels
        ax1.set_xlabel("X Position")
        ax1.set_ylabel("Y Position")
        ax2.set_xlabel("X Position")
        ax2.set_ylabel("Y Position")

        # Views
        ax2.view_init(elev=90, azim=-90)
        ax2.w_zaxis.line.set_lw(0.0)
        ax2.set_zticks([])

        # Animate
        def animate(i, objects):
            objects[0].remove()
            objects[1].remove()
            p, r = self.get_path_info(i)
            self.robot.transform(p, r)
            objects[0] = self.robot.plot(ax1, "blue")
            objects[1] = self.robot.plot(ax2, "blue")
            c_path = self.pos[0 : (i + 1), :]
            objects[2].set_xdata(c_path[:, 0])
            objects[2].set_ydata(c_path[:, 1])
            objects[2].set_3d_properties(c_path[:, 2])

            objects[3].set_xdata(c_path[:, 0])
            objects[3].set_ydata(c_path[:, 1])
            objects[3].set_3d_properties(c_path[:, 2])
            return objects

        l1 = ax1.plot([], [], [], color="k")
        l2 = ax2.plot([], [], [], color="k")

        objects = [
            self.robot.plot(ax1, "blue"),
            self.robot.plot(ax2, "blue"),
            l1[0],
            l2[0],
        ]

        ani = FuncAnimation(
            fig,
            animate,
            fargs=(objects,),
            frames=self.n,
            interval=100,
            blit=True,
        )
        # Save
        ani.save(
            "test.gif",
            writer="pillow",
            fps=1,
        )
        # plt.show()


obs = Cuboid(np.array([5, 0, 0]), np.array([5, 5, 5]), np.array([0, 0, 0, 1]))
robot = Cuboid(np.array([0, 0, 0]), np.array([2, 1, 1]), np.array([0, 0, 0, 1]))

files = [f for f in listdir("solutions") if isfile(join("solutions", f))]
for fname in files:
    print(f"Working on {fname}...")
    SA = SolutionAnimator(obs, robot, fname, "solutions/", "visualization/")
    SA.SubplotAnimate()
    # SA.AzimuthAnimate()
