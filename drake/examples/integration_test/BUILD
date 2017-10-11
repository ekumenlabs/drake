# -*- python -*-

load(
    "//tools:drake.bzl",
    "drake_cc_googletest",
    "drake_cc_library",
    "drake_cc_binary",
)
load("//tools:install_data.bzl", "install_data")
load("//tools/lint:lint.bzl", "add_lint_tests")

drake_cc_binary(
    name = "integration_test",
    srcs = [
        "sample.cc",
    ],
    deps = [
        "//drake/automotive/maliput/multilane:multilane",
        "//drake/systems/analysis",
        "//drake/systems/framework:leaf_system",
        "//drake/systems/primitives:matrix_gain",
    ],
)
