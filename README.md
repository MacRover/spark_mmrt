# MMRT SparkMAX Library
MMRT's CAN Library for controlling REV Robotics SparkMAX motor controller


current instructions to run control.cpp

g++ -std=c++17 -I./spark_mmrt/include   ./spark_mmrt/examples/control.cpp   ./spark_mmrt/src/can/SocketCanTransport.cpp   ./spark_mmrt/src/device/SparkMax.cpp   ./spark_mmrt/src/frames/SparkFrames.cpp   -o control

g++ -std=c++17 -I./spark_mmrt/include   ./spark_mmrt/examples/keyboard_wd_motor17.cpp   ./spark_mmrt/src/can/SocketCanTransport.cpp   ./spark_mmrt/src/device/SparkMax.cpp   ./spark_mmrt/src/frames/SparkFrames.cpp   -o keyboard_wd_motor17


then 
./control
