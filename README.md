# quadson_py

## Installation

For Docker users, use the following command to build and run the environment.

```sh
make
```

For python venv users, activate your new virtual environment and choose the requirements file that fits your needs:

```sh
# CAN bus support
pip install -r requirements-can.txt
```

```sh
# Simulation support
pip install -r requirements-sim.txt
```

Then install the project to ensure that the modules can be imported correctly:

```sh
pip install -e .
```

Before launching the program, please make sure your host kernel supports CAN bus. Otherwise, you may encounter errors such as:

```sh
modprobe: FATAL: Module can not found in directory /lib/modules/6.12.28-1-lts
```

or

```sh
modprobe: ERROR: could not insert 'can': Exec format error
modprobe: ERROR: could not insert 'can_raw': Exec format error
```

or

```sh
Cannot find device "can0"
```

To check CAN support on your system, run the following command:

```sh
ls /lib/modules/$(uname -r)/kernel/net/can
```

You should see something like:

```sh
can.ko  can-dev.ko  can-raw.ko
```

If not, your kernel may not have CAN bus support built-in, and you will need to switch to a compatible kernel or manually build CAN modules.

To manually load CAN modules:

```sh
sudo modprobe can
sudo modprobe can_raw
sudo modprobe can_dev
```

After loading the modules, restart the program. This should resolve the issue.
