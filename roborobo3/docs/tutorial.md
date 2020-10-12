# Tutorial



#



#

#
## How to add functionalities

All the code to bind python functionalities are in `src/ext/pyroborobo/`.


## Common issues

Lot of segfault errors are because of `dynamic_cast` from c++ code, by ommitting to call the `super.__init__` when overloading an object in python or by not having the config and data folder in your python project (not in roborobo package).

