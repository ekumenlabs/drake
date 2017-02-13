To compile:

```
bazel build drake/sdf_sample:sample
```

To run and see it on the visualizer:

```
build/install/bin/drake-visualizer &
./bazel-bin/drake/sdf_sample/sample
```