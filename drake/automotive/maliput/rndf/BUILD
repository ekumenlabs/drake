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
        "branch_point.cc",
        "junction.cc",
        "lane.cc",
        "road_geometry.cc",
        "segment.cc",
        "spline_lane.cc",
    ],
    hdrs = [
        "branch_point.h",
        "junction.h",
        "lane.h",
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

filegroup(
    name = "yamls",
    srcs = glob(["*.yaml"]),
)

# === test/ ===

drake_cc_googletest(
    name = "rndf_builder_test",
    size = "small",
    srcs = ["test/rndf_builder_test.cc"],
    deps = [
        ":builder",
    ],
)

drake_cc_googletest(
    name = "rndf_lanes_test",
    size = "small",
    srcs = ["test/rndf_lanes_test.cc"],
    deps = [
        ":lanes",
        "//drake/common:eigen_matrix_compare",
    ],
)

drake_cc_binary(
    name = "yaml_load",
    testonly = 1,
    srcs = ["test/yaml_load.cc"],
    deps = [
        ":loader",
        "//drake/common:text_logging_gflags",
    ],
)

py_test(
    name = "yaml_load_test",
    size = "small",
    srcs = ["test/yaml_load_test.py"],
    args = ["drake/automotive/maliput/rndf/yaml_load"],
    data = [
        ":yaml_load",
        ":yamls",
    ],
)

cpplint()
