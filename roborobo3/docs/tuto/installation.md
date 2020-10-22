# Installation

## Linux

First, download the project. make sure you have git and make tools

```bash
sudo apt install git build-essential cmake 
```

```bash
cd ~/Documents/MyPyroboroboRepo
git clone -b cleanpyroborobo https://github.com/PaulEcoffet/roborobo3.git
```

Then, we must compile pyroborobo with the python interpreter that *you* intend to use for your own projects. You may want to create a new conda environment.

```bash
conda create --name pyrob numpy pybind11
conda activate pyrob
```

The dependencies are numpy and pybind11 on the python side. For the c++ side, you need SDL2 and boost.

If you have not done it with conda before (or pip if you don’t use conda):

```bash
conda install pybind11 numpy
# or with pip
# pip install -U pybind11
# pip install -U numpy
```

Then, you must install some dependencies from your package manager. Here are the one from ubuntu:

```bash
sudo apt-get install libsdl2-dev libsdl2-image-dev libboost-dev libeigen-dev
```



Then, to install pyroborobo, run in the pyroborobo directory:

```
python setup.py install --force
```

## Mac OS

First, download the project :

```bash
cd ~/Documents/MyPyroboroboRepo
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