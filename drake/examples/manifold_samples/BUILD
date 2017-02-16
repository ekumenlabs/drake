package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "manifold_sample",
    srcs = [
        "src/main.cc",
    ],
    linkopts = [
	"-lignition-math2",
        "-lmanifold0",
    ],
    copts = [
	"-I/usr/include/ignition/math2",
	"-I/usr/include/manifold0",
	"-std=c++11"
    ],
    deps = [
    ], 
)
