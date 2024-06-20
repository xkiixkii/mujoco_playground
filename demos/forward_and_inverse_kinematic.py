import mujoco
from robot_descriptions.loaders.mujoco import load_robot_description
import numpy as np

if __name__ == '__main__':
    # Load the model
    model = load_robot_description("ur10e_mj_description")
    data = mujoco.MjData(model)