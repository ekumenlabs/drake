# -*- python -*-

load("//tools:cpplint.bzl", "cpplint")
load(
    "//tools:drake.bzl",
    "drake_cc_googletest",
    "drake_cc_library",
    "drake_cc_binary",
)

drake_cc_binary(
    name = "spline_comparison",
    deps = [
        "//drake/common:text_logging_gflags",
        "//drake/common",
        "//drake/common/trajectories:piecewise_polynomial",
        "@ignition_math",
    ],
    srcs = ["spline_comparison.cc"],
)

# === test/ ===

cpplint()