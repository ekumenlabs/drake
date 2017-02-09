To compile:

```
bazel build drake/sdf_sample:sample
```

To run and see it on the visualizer:

```
./build/install/bin/drake_visualizer &
./bazel-bin/drake/sdf_sample/sample <path_to_sdf>
```