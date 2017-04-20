# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

load("//tools:cpplint.bzl", "cpplint")
load("//tools:drake.bzl", "drake_cc_binary", "drake_cc_googletest", "drake_cc_library")

package(default_visibility = ["//visibility:public"])

drake_cc_library(
    name = "rndf",
    srcs = [],
    hdrs = [],
    deps = [
        ":loader",
    ],
)

drake_cc_library(
    name = "lanes",
    srcs = [
        "branch_point.cc",
        "junction.cc",
        "lane.cc",
        "road_geometry.cc",
        "segment.cc",
        "spline_lane.cc",
        "spline_helpers.cc"
    ],
    hdrs = [
        "branch_point.h",
        "junction.h",
        "lane.h",
        "road_geometry.h",
        "segment.h",
        "spline_lane.h",
        "spline_helpers.h",
    ],
    deps = [
        "//drake/automotive/maliput/api",
        "//drake/common",
        "//drake/math:geometric_transform",
        "@ignition_math",
    ],
)

drake_cc_library(
    name = "builder",
    srcs = [
        "builder.cc",
    ],
    hdrs = [
        "builder.h",
    ],
    deps = [
        ":lanes",
        "@ignition_math",
    ],
)

drake_cc_library(
    name = "loader",
    srcs = ["loader.cc"],
    hdrs = ["loader.h"],
    deps = [
        "//drake/automotive/maliput/api",
        ":builder",
        "@ignition_math",
        "@ignition_rndf",
    ],
)

filegroup(
    name = "rndfs",
    srcs = glob(["maps/*.rndf"]),
)

# === test/ ===

drake_cc_googletest(
    name = "ignition_math_test",
    srcs = ["test/ignition_math_test.cc"],
    deps = [
        "@ignition_math",
    ],
)

cpplint()
