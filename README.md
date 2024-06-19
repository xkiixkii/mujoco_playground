# MJ-Playground

***MuJoCo Version: 3.1.6
Python Version: 3.11.9
OS: Ubuntu 22.04.4 x86_64***

Some painful learning experiences, I hope that helps a little bit :).
More details see [MuJoCo Documentation](https://mujoco.readthedocs.io/en/stable/python.html#)

## Preparation

```bash
conda env create -f environment.yml
conda activate mj
```

## Some Useful APIs

- **Static factory functions that create a new `mujoco.MjModel`  instance**
  - `mujoco.MjModel.from_xml_path(path)`: create from a xml file
  - `mujoco.MjModel.from_xml_string(xml_string)`: create from a xml string
- **Functions**
  - `mujoco.mj_step(model, data)`
- **Passive viewer**
  - The `mujoco.viewer.launch_passive(model, data, *, key_callback=None,show_left_ui=True, show_right_ui=True)` returns a handle which can be used to interact with the viewer. It can be used as a context manager.
  - Arguments:
    - **model:**  MjModel instance.
    - **data:** MjData.
    - **key_callback:** A callable which gets called each time a keyboard event occurs in the viewer window e.g. pause the movement (see example above)
    - **show_left_ui:** if show left ui.
    - s**how_right_ui:** if show right ui.
  - Attributes (incomplete):
    - `viewer.is_running()`: check viewer window is still running.
    - `viewer.sync()`: update MjModel and MjData render the model after a physic step.(call after `mujoco.mj_step()`)
    - `viewer.close()`: call after you are done with the viewer, if you do not use the viewer as a context manager.
    - `cam`, `opt`, and `pert` properties: correspond to `mjvCamera`, `mjvOption`, and `mjvPerturb` structs, respectively.
    - 
## 1. Load a model from URDF or MJCF file and render it

Example:
```python
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
    # Here we load the UR10e robot description 
    # via robot_descriptions package
    model = load_robot_description("ur10e_mj_description")
    # Or you can load the robot description directly from a file
    # model = mujoco.MjModel.from_xml_file("your_own_robot_description.xml")

    # Load MjData
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

```