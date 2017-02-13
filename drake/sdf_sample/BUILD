package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "sample",
    srcs = [
        "src/sample.cc",
    ],
    data = [
        ":models",
    ],
    deps = [
        "//drake/lcm",
        "//drake/multibody/parsers",
        "//drake/multibody/rigid_body_plant:drake_visualizer",
        "//drake/multibody/rigid_body_plant",
        "//drake/systems/analysis",
        "//drake/systems/lcm",
        "//drake/systems/primitives:constant_vector_source",
        "//drake/systems/primitives:multiplexer",
        "//drake/common:text_logging_gflags",
        "//drake/systems/controllers:pid_controlled_system",
    ],
)

filegroup(
    name = "models",
    srcs = glob(["models/**"]),
)