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
        ":builder",
        ":lanes",
        ":loader",
    ],
    copts = [
        "-I/usr/include/ignition/math2",
        "-std=c++11"
    ],
    linkopts = [
        "-lignition-math2",
    ],
)

drake_cc_library(
    name = "lanes",
    srcs = [
        "arc_lane.cc",
        "branch_point.cc",
        "junction.cc",
        "lane.cc",
        "line_lane.cc",
        "road_geometry.cc",
        "segment.cc",
        "spline_lane.cc",
    ],
    hdrs = [
        "arc_lane.h",
        "branch_point.h",
        "junction.h",
        "lane.h",
        "line_lane.h",
        "road_geometry.h",
        "segment.h",
        "spline_lane.h",
    ],
    copts = [
        "-I/usr/include/ignition/math2",
        "-std=c++11"
    ],
    linkopts = [
        "-lignition-math2",
    ],
    deps = [
        "//drake/automotive/maliput/api",
        "//drake/common",
        "//drake/math:geometric_transform",
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
    ],
    copts = [
        "-I/usr/include/ignition/math2",
        "-std=c++11"
    ],
    linkopts = [
        "-lignition-math2",
    ],
)

drake_cc_library(
    name = "loader",
    srcs = [
        "loader.cc",
    ],
    hdrs = [
        "loader.h",
    ],
    deps = [
        ":builder",
        "@yaml_cpp//:lib",
    ],
)

filegroup(
    name = "yamls",
    srcs = glob(["*.yaml"]),
)

# === test/ ===


cpplint()
