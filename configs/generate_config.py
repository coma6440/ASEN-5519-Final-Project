import yaml
import numpy as np

data = {
    "obstacles": {
        "obs1": {
            "position": [6, 5, 5],
            "orientation": [0.0, 0.0, 0.0, 1.0],
            "size": [1.0, 1.0, 1.0],
        },
        "obs2": {
            "position": [10, -7, -5],
            "orientation": [0.0, 0.0, 0.0, 1.0],
            "size": [1.0, 1.0, 1.0],
        },
        "obs3": {
            "position": [3, 2, 1],
            "orientation": [0.0, 0.0, 0.0, 1.0],
            "size": [1.0, 1.0, 1.0],
        },
        "obs4": {
            "position": [11, -1, 7],
            "orientation": [0.0, 0.0, 0.0, 1.0],
            "size": [1.0, 1.0, 1.0],
        },
        "obs5": {
            "position": [1, -4, -6],
            "orientation": [0.0, 0.0, 0.0, 1.0],
            "size": [1.0, 1.0, 1.0],
        },
    }
}

data["robot"] = {
    "size": [2.0, 1.0, 1.0],
    "start_position": [0.0, 0.0, 0.0],
    "goal_position": [15.0, 15.0, 15.0],
    "start_orientation": [0.0, 0.0, 0.0, 1.0],
}
n_obs = 30

for i in range(n_obs):
    pos = list(np.random.randint(low=[0, -10, -10], high=[15, 10, 10]))
    data["obstacles"]["obs" + str(i)] = {
        "orientation": [0.0, 0.0, 0.0, 1.0],
        "position": [float(i) for i in pos],
        "size": [1.0, 1.0, 1.0],
    }
with open("configs/w5.yaml", "w") as file:
    yaml.dump(data, file)
