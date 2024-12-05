sudo modprobe can
sudo modprobe can_raw
sudo modprobe mttcan
sudo ip link set can0 type can bitrate 1000000
sudo ip link set up can0

sudo chmod 777 /dev/ttyTHS0
../build/RM_exe
