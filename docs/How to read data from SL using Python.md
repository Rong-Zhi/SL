How to read data from SL using Python
-------------------------------------

A convenient way of working with SL is to run a simulation written in C/C++ and
then analyze it using a scripting language like Python or Matlab.
This tutorial assumes that you already know
[How to record data in SL](docs/How%20to%20record%20data%20in%20SL.md).
Here you will learn how to read and visualize the data from files created by SL.

Firstly, obtain Python 3, Jupyter, matplotlib, and ipywidgets.
If you are new to Python, consult
[The Hitchhiker’s Guide to Python](http://docs.python-guide.org/en/latest/).

### Read the data
* Start an IPython notebook
* Import required modules

```python
# Plotting
%matplotlib inline
import matplotlib.pyplot as plt
from ipywidgets import interact

# System
import os.path
import struct
from typing import Tuple

# Numerics
import numpy as np
np.set_printoptions(precision=4, suppress=True)
```

* Define a function that reads the data

```python
def read_SL_data(file_name: str) -> Tuple[np.ndarray, float]:
    with open(file_name, 'rb') as f:
        # Extract data size
        header = f.readline()
        data_sizes = header.rstrip(b'\n').split()
        num_total = int(data_sizes[0])
        num_cols  = int(data_sizes[1])
        num_rows  = int(data_sizes[2])
        freq      = float(data_sizes[3])
        # Extract variable names and units
        variables = f.readline()
        var_names_and_units = variables.rstrip(b'\n').split()
        var_names = [var_names_and_units[2*i].decode() for i in range(num_cols)]
        var_units = [var_names_and_units[2*i+1].decode() for i in range(num_cols)]
        # Read the data table
        size_of_float = 4
        var_dtype = [(var_names[i], '>f' + str(size_of_float)) for i in range(num_cols)]
        d = np.zeros((num_rows,), dtype=var_dtype)
        for i in range(num_rows):
            x = f.read(num_cols*size_of_float)
            d[i] = struct.unpack('>' + str(num_cols) + 'f', x)
    return d, freq
```

* Read the data

```python
# Read the data
robot_path = '/home/belousov/.CLion2016.2/system/cmake/generated/badminton-5d36eb34/5d36eb34/Debug/darias/'
file_name = 'd00018'
file_path = os.path.join(robot_path, file_name)
d, fs = read_SL_data(file_path)
```

The variable `d` now contains all the information. You can examine what variables
have been recorded in `d.dtype.names` and plot them.
For instance, you can plot the position of the `R_SFE` joint over time

```python
plt.plot(d['time'], d['R_SFE_th'], lw=2)
```
