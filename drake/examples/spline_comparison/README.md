# Spline comparison

Here we compare ignition::math::Spline with Pchip and Cubic functions from drake/common/trajectories


We compare the result of 0, 1, 2 derivative orders of the interpolation.

As the tests show, you'll have almost no difference between ignition::math::Spline and Cubic interpolation if you set the tangents to the knots.

## How to build it?

```
bazel build drake/examples/spline_comparison
```

## How to run it?

```
bazel run drake/examples/spline_comparison
```

You should expect a bunch of numbers printed on the screen. Those are the results.