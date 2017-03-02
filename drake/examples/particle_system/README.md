# Particle sample

## Description

This is a sample project that creates a minimalistic system of a particle which knows how to move with 1 DOF. You can pass it the initial conditions of the system, which are:

* xi: initial position in meters
* vi: initial speed in meters per second

Also, you can set the final time of the simulation, and the acceleration of the particle (this solves a constant accelarated particle). At the end of execution you should get the final position and speed.

## How do I compile it?

To compile this project just run (from Drake's repository path):

```
bazel compile drake/examples/particle_system:sample
```

## How do I run it?

To run this project just run (from Drake's repository path):

```
bazel run drake/examples/particle_system:sample [t_final] [x_init] [v_init] [accel]
```