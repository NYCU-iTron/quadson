modprobe can
modprobe can_raw
modprobe can_dev
ip link set can0 type can bitrate 500000 sample-point 0.875
ip link set up can0