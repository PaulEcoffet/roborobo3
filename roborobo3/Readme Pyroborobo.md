# Readme Pyroborobo



## Installation

First, download the project :

```bash
cd ~/Documents/MyPyroboroboRepo
git clone -b cleanpyroborobo https://github.com/PaulEcoffet/roborobo3.git
```

Then, we must compile pyroborobo with the python interpreter that *you* intend to use for your own projects. You may want to create a new conda environment.

```bash
conda create --name pyrob numpy pybind11 ipython
conda activate pyrob
```

The dependencies are numpy and pybind11 on the python side. For the c++ side, you need SDL2 and boost.

If you have not done it with conda before:

```bash
conda install pybind11 numpy
# or with pip
# pip install -U pybind11
# pip install -U numpy
```

I use the SDL2 from brew, it *might* work with the .framework. If it does not :

```bash
brew install sdl2
brew install sdl2-image
brew install boost
```



Then, to install pyroborobo, run

```
python setup.py install --force
```



## How to use pyroborobo



Here are two very simple examples:

```python
from pyroborobo import Pyroborobo

#Pyroborobo.__init__(self: pyroborobo.Pyroborobo, properties_file: str, world_observer_class: object, controller_class: object, world_model_class: object, agent_observer_class: object, override_conf_dict: dict)
# 1e param : conf file for roborobo
# 2n param : controller class in python
# 3e params : controller class in python, if None, use cpp class, if "dummy" use a dummy cpp class which does nothing
# 4e : worldmodel class en python, pareil que pour controller
# quatrièr

rob = Pyroborobo("config/template_randomwalk.properties", None, None, None, None, {})
rob.start()
stp = False
while not quit:
	stop = rob.update(1000) # update for 1000 time step before going back in python mode, except if quit early
rob.close()
```



Second more complex example:



```python
from pyroborobo import Pyroborobo, PyController
import numpy as np


class MyController(PyController):
    def __init__(self, wm):
        PyController.__init__(self, wm)  # NEEDED otherwise segfault, pybind limitation
        self.wm = wm

    def reset(self):
        pass

    def step(self):
        sensors = self.wm.getCameraSensorsDist()
        if np.all(sensors[2:7] > 0.5):
            self.wm.speed = 1
            self.wm.rotspeed = 0
        else:
            self.wm.speed = 0
            self.wm.rotspeed = 20


if __name__ == "__main__":
    # Use a dummy world observer that doesn't call any dynamic_cast
    rob = Pyroborobo("config/template_randomwalk.properties", "dummy", MyController, None, None, {})
    rob.start()
    stop = False
    while not stop:
        stop = rob.update(1000)  # update for 1000 time step before going back in python mode, except if quit early
    rob.close()
```



## How to add functionalities

All the code to bind functionnalities are in `src/ext/pyroborobo/pyroborobo.cpp`.



## Common issues

Lot of segfault errors are because of `dynamic_cast` from c++ code, by ommitting to call the `super.__init__` when overloading an object in python or by not having the config and data folder in your python project (not in roborobo package).

