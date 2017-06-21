# -*- python -*-

load("//tools:cpplint.bzl", "cpplint")
load(
    "//tools:drake.bzl",
    "drake_cc_googletest",
    "drake_cc_library",
    "drake_cc_binary",
)

drake_cc_binary(
    name = "bezier_curve",
    deps = [
        "//drake/common",
        "@ignition_math",
    ],
    srcs = ["bezier_curve.cc"],
)

# === test/ ===

cpplint()