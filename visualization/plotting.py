import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from os import listdir
from os.path import isfile, join, exists
from scipy.spatial import ConvexHull
from scipy.spatial.transform import Rotation as R
import yaml


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
        self.rot = data[:, 3:7]

    def get_path_info(self, idx):
        return self.pos[idx, :], self.rot[idx, :]

    def AzimuthAnimate(self):
        fig = plt.figure()
        ax = plt.axes(projection="3d")
        for o in self.obs:
            o.plot(ax, "red")
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
        fig.subplots_adjust(left=0, right=1, bottom=0, top=1)

        # Add obstacles
        for o in self.obs:
            o.plot(ax1, "red", 0.3)
            o.plot(ax2, "red", 0.3)

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
            interval=1,
            blit=True,
        )
        # Save
        ani.save(
            self.out_folder + self.fname.split(".")[0] + ".gif",
            writer="pillow",
            fps=10,
        )
        # plt.show()


def load_environment(fname):
    with open("configs/" + fname, "r") as f:
        data = yaml.safe_load(f)
        yaml_obs = data["obstacles"]
        yaml_rob = data["robot"]
        robot = Cuboid(
            yaml_rob["start_position"], yaml_rob["size"], yaml_rob["start_orientation"]
        )
        obstacles = []
        for key in yaml_obs:
            obs = yaml_obs[key]
            obstacles.append(Cuboid(obs["position"], obs["size"], obs["orientation"]))
        return robot, obstacles


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
for fname in geo_files:
    # TODO: File naming and separating into folders
    print(f"Working on geometric solution: {fname}...")
    robot, obstacles = load_environment(fname.split("_")[0] + ".yaml")
    SA = SolutionAnimator(
        obstacles, robot, fname, "solutions/geometric/", "visualization/"
    )
    if not exists(SA.out_folder + SA.fname.split(".")[0] + ".gif"):
        SA.SubplotAnimate()
    else:
        print("Solution plot already exists! Moving on...")

for fname in kino_files:
    # TODO: File naming and separating into folders
    print(f"Working on kinodynamic solution: {fname}...")
    robot, obstacles = load_environment(fname.split("_")[0] + ".yaml")
    SA = SolutionAnimator(
        obstacles, robot, fname, "solutions/kinodynamic/", "visualization/"
    )
    if not exists(SA.out_folder + SA.fname.split(".")[0] + ".gif"):
        SA.SubplotAnimate()
    else:
        print("Solution plot already exists! Moving on...")
