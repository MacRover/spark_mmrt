# MMRT SparkMAX Library
MMRT's Library for controlling REV Robotics SparkMAX motor controller, supported on firmware versions 25.0.X. This has been tested and confirmed to work on 25.0.4.

This repository also contains example applications that test the library's APIs extensively as well as useful interfaces for servicing and debugging SparkMAX devices through CAN. You can find these in the `examples/` directory.

## Building
To compile all of the example applications, run `make` in the base directory. You can also build a specific target by passing the name as an argument:

```bash 
make <TARGET_NAME>
# build control.cpp for example
make control
```
Executables are stored in the `bin/` folder.