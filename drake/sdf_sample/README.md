### sdf_sample

## Description


## To compile:

```
bazel build drake/sdf_sample:sample
```

## To compile for debugging purposes:

```
bazel build drake/sdf_sample:sample -c dbg
```

## To run and see it on the visualizer:

```
build/install/bin/drake-visualizer &
./bazel-bin/drake/sdf_sample/sample
```