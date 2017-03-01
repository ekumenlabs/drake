# Maliput sample

## Description

This is a sample program that opens a Monolane file and prints all its junctions, segments, lanes and branch points.

## To build

From the base repository directory:

```
bazel build drake/examples/maliput_sample:sample
```

## To run

From the base repository directory:

```
bazel run drake/examples/maliput_sample:sample </full/path/to/monolane/file.yaml>
```
