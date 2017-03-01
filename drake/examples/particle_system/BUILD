cc_library(
    name = "particle",
    srcs = ["src/particle.cc"],
    hdrs = [
        "src/particle.hh",
        "src/particle-inl.hh",
    ],
    deps = [
        "//drake/common",
        "//drake/systems/framework:leaf_system",
    ],
)

cc_binary(
    name = "sample",
    srcs = ["src/main.cc",
    ],
    deps = [
        ":particle",
        "//drake/systems/analysis",
    ],
)