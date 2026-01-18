# Simulation

## Enable X11 on host
In a terminal on host machine enable the container to connect to X11 by launching:
```
xhost +local:
```
you can undo this later with a restart or with:
```
xhost -local:
```


## How to launch Sim

In a terminal launch:
```
bazel run //simulation:launch_sim
```

### Foxglove bridge

Launch foxglove and connect to `ws://localhost:8765`.
