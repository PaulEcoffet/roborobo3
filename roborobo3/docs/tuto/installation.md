# Installation

## Linux

First, download the project. make sure you have git and make tools

```bash
sudo apt install git build-essential cmake 
```

```bash
cd <yourpyroborobofolder>
git clone -b cleanpyroborobo https://github.com/PaulEcoffet/roborobo3.git
```

Then, we must compile pyroborobo with the python interpreter that *you* intend to use for your own projects. You may want to create a new conda environment.

```bash
conda create --name pyrob numpy pybind11 setuptools
conda activate pyrob
```

The dependencies are numpy and pybind11 on the python side. For the c++ side, you need SDL2 and boost.

If you have not done it with conda before (or pip if you don’t use conda):

```bash
conda install numpy setuptools
conda install -c conda-forge pybind11  # install latest pybind11
# or with pip
# pip install -U pybind11
# pip install -U numpy
```

Then, you must install some dependencies from your package manager. Here are the one from ubuntu:

```bash
sudo apt-get install libsdl2-dev libsdl2-image-dev libboost-dev libeigen3-dev
```

Then, to install pyroborobo, run in the pyroborobo directory:

```bash
cd roborobo3/roborobo3
# if you don't have activated your virtual env
conda activate pyrob
python setup.py install --force
```

The documentation is available by opening `docs/_build/html/index.html`, you can do it using this shell command:

```bash
xdg-open <yourpyroborobofolder>/roborobo3/roborobo3/docs/_build/html/index.html
```


You can also build the docs yourself:

```bash
conda activate pyrob
conda install sphinx recommonmark sphinx_rtd_theme numpydoc

cd <yourpyroborobofolder>/roborobo3/roborobo3/docs
make clean && make html
open <yourpyroborobofolder>/roborobo3/roborobo3/docs/_build/html/index.html
```


Many examples are available
on [github in the py_example folder](https://github.com/PaulEcoffet/roborobo3/tree/cleanpyroborobo/roborobo3/py_example).

## Mac OS

First, download the project :

```bash
cd <yourpyroborobofolder>
git clone -b cleanpyroborobo https://github.com/PaulEcoffet/roborobo3.git
```

Then, we must compile pyroborobo with the python interpreter that *you* intend to use for your own projects. You may want to create a new conda environment.

```bash
conda create --name pyrob numpy pybind11
conda activate pyrob
```

The dependencies are numpy and pybind11 on the python side. For the c++ side, you need SDL2 and boost.

If you have not done it with conda before:

```bash
conda install numpy setuptools
conda install -c conda-forge pybind11  # install latest pybind11
# or with pip
# pip install -U pybind11
# pip install -U numpy
```

I use the SDL2 from brew, it *might* work with the .framework. If it does not :

```bash
brew install sdl2
brew install sdl2_image
brew install boost
brew install eigen
brew install cmake
```



Then, to install pyroborobo, run

```
cd roborobo3/roborobo3
python setup.py install --force
```


The documentation is available by opening `docs/_build/html/index.html`, you can do it using this shell command:

```bash
open <yourpyroborobofolder>/roborobo3/roborobo3/docs/_build/html/index.html
```

You can also build the docs yourself:

```bash
conda activate pyrob
conda install sphinx recommonmark sphinx_rtd_theme numpydoc

cd <yourpyroborobofolder>/roborobo3/roborobo3/docs
make clean && make html
open <yourpyroborobofolder>/roborobo3/roborobo3/docs/_build/html/index.html
```

Many examples are available
on [github in the py_example folder](https://github.com/PaulEcoffet/roborobo3/tree/cleanpyroborobo/roborobo3/py_example).
