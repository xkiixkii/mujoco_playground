import mujoco
import mujoco.viewer
import numpy as np
from robot_descriptions.loaders.mujoco import load_robot_description

pause: bool = False


def key_callback(kw):
    if chr(kw) == " ":
        global pause
        pause = not pause


if __name__ == "__main__":
    # Here we load the UR10e robot description via robot_descriptions package
    model = load_robot_description("ur10e_mj_description")
    # Or you can load the robot description directly from a file
    # model = mujoco.MjModel.from_xml_file("your_own_robot_description.xml")

    # Load MjData class
    data = mujoco.MjData(model)
    # Launch the viewer
    with mujoco.viewer.launch_passive(
        model, data, key_callback=key_callback, show_left_ui=False, show_right_ui=False
    ) as viewer:
        while viewer.is_running():
            if not pause:
                # Randomly set the control input
                data.ctrl = np.random.uniform(-10, 10)
                # Step the simulation
                mujoco.mj_step(model, data)
                # Sync the data to the viewer and render
                viewer.sync()
