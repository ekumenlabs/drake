# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

load("//tools:cpplint.bzl", "cpplint")
load("//tools:drake.bzl", "drake_cc_binary", "drake_cc_googletest", "drake_cc_library")

package(default_visibility = ["//visibility:public"])

drake_cc_library(
    name = "oneway",
    srcs = [
        "branch_point.cc",
        "junction.cc",
        "lane.cc",
        "road_geometry.cc",
        "segment.cc",
    ],
    hdrs = [
        "branch_point.h",
        "junction.h",
        "lane.h",
        "road_geometry.h",
        "segment.h",
    ],
    deps = [
        "//drake/automotive/maliput/api",
        "//drake/common",
    ],
)

drake_cc_binary(
    name = "oneway_to_urdf",
    srcs = ["oneway_to_urdf.cc"],
    deps = [
        ":oneway",
        "//drake/automotive/maliput/utility",
        "//drake/common",
        "//drake/common:text_logging_gflags",
        "//drake/thirdParty:spruce",
        "@gflags//:gflags",
    ],
)

# === test/ ===

drake_cc_googletest(
    name = "oneway_test",
    size = "small",
    srcs = ["test/oneway_test.cc"],
    deps = [
        ":oneway",
    ],
)

cpplint()
