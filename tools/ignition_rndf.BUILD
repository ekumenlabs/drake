# -*- python -*-

cc_library(
    name = "ignition_rndf",
    srcs = glob(
        ["src/*.cc"],
        exclude = ["src/*_TEST.cc"],
    ),
    hdrs = glob([
        "include/**/*.hh",
    ]),
    deps = [
        "@ignition_math//:ignition_math"
    ],
    includes = ["include"],
    visibility = ["//visibility:public"],
)
