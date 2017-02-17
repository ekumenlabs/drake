### sdf_sample

## Description

This sample program is capable of loading a SDF file into Drake. It has a sample model file on
its models folder and loads it as a default or you can choose to visualize your own model.

## Compilation

To just compile:

```
bazel build drake/sdf_sample:sample
```

To compile for debugging purposes:

```
bazel build drake/sdf_sample:sample -c dbg
```

## Running

You can make use of the run.sh script. So from this folder just type:

```
./run.sh
```

This program can be invoked like:

```
sample </path/to/sdf_file.sdf>
```

Where:

* `/path/to/sdf_file.sdf` is an optional argument. If not provided, the program will try to load a sample SDF file on it.

## Special considerations

If you use obj mesh files, consider your locale configuration for the floating point separator. We recommend using '.' and configuring your environment with `LC_ALL=C` before running. To do so:

```
export LC_ALL=C
```

An then continue with the **Running** stage.