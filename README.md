# orion-ros2-reference

## Setup cross compilation

The cross compilation setup is done within the build_dev docker image.
The high level setup is:

- build a development docker image
- setup the target system
- create a sysroot from the target system
- run docker and mount repository and sysroot
- cross compile ros2
- cross compile orion

### Setup target

#### Setup key authentication with the target

This is needed for the create_sysroot script to not ask for ssh password several times.

```
$ ssh-copy-id user@target-ip
```

yadda yadda

### Create sysroot from target

The sysroot can be created from a readily setup target system by using the
`./docker/create_sysroot.sh` script.

NOTE: This script relies on ssh key authentication being setup between host and target.

The script takes two arguments

- ssh connection string to the target (ex. user@ip)
- target directory for the sysroot on the host system

```
$ ./docker/create_sysroot.sh user@192.168.55.1 /home/user/code/sysroots/xavier-orion-sysroot
```

This will start to collect the relevant files from the target and prepare a sysroot for cross
compilation at the specified path.

### Crosscompile ros2

Switch into the docker environment and make sure to mount source repository as well as sysroot correctly

```
# assummes the following environment vars to be set
# WS_KROS=/path/to/orion/repo
# ORION_SYSROOT=/path/to/sysroot

$ docker run -it -v $WS_KROS:/opt/ws_kros -v $ORION_SYSROOT:/opt/sysroot --rm orion:dev

# invoke compile script from docker env
root@docker $ ./docker/run_in_docker_scripts/build_arm_ros2.sh
```

At the end of the compilation process, make sure that no packages has been skipped.

### Crosscompile orion

Make sure that compilation of ros2 succeeded and call from within the docker env

```
root@docker $ ./docker/run_in_docker_scripts/build_arm.sh
```

### Deploy to target
