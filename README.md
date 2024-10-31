# RPLidar Python Module

This repository adapts the c++ Slamtec SDK module and converts it into a python module for Slamtec RPLidars S models.

## Table of Contents

- [Requirements](#Requirements)
- [Easy start](#Easy-start)
- [Compiling the Module](#Compiling-the-module)
- [Class API](#Class-API)
- [Usage](#usage)
- [Examples](#examples)

## Requirements
- CMake 3.14+
- Pybind11 2.11+
- Pygame
## Easy Start

Download the precompiled `.so` module and the file `lidar_test.py` to your working folder, and go to [Usage](#usage).

## Compiling the Module

1. Clone this repository
```bash
git clone --recurse-submodules https://github.com/FIU-MoRA-Lab/rplidar_python.git
cd rplidar_python
#    git submodule update --init --recursive

```

2. Compile the Slamtec SDK by typing 
```bash
make -C /rplidar_sdk
``` 

3. Compile the Python module by running
```bash
cmake -S . -B build -DCMAKE_LIBRARY_OUTPUT_DIRECTORY=../  && cmake --build build && rm -rf build
```
You should end up with a `.so` file whose name starts by `rplidar.cpython`.

## Class API

### _class_ `rplidar.LidarWrapper(port = '/dev/ttyUSB0', baudrate = 1000000)`
Basic wrapper managing communication with the RPLidar

**Parameters:**
- **port** (`str`): The serial port to which the RPLIDAR is connected (default: `/dev/ttyUSB0`).
- **baudrate** (`int`): The baud rate for the serial communication (default: `1000000`).


### Attributes
---

The class has no accesible attributes

### Methods
---

##### `get_scan_data()`
Get scan data from the Lidar

**Returns:**
- **measurements** (`list[float]`): A list of 360 float values (one per degree) representing distance measurements in meters

##### `initialize()`
Initialize the Lidar.

**Returns:** 
- **is_initialized**`(`bool`): `True` if the Lidar is successfully initialized, `False` otherwise

##### `start_scan()`
Start scanning with the Lidar

**Returns:** 
- (`None`): `None`

##### `stop_scan()`
Stop scanning with the Lidar

**Returns:**
-  (`None`): `None`

## Usage

To import the class is enough to do a common import statement
```
import rplidar
port = '/dev/ttyUSB0'
baudrate = 1000000
rplidar_instance = rplidar.LidarWrapper(port, baudrate)
```

The example can be run by using the test file `lidar_test.py`

```
python3 lidar_test.py
```