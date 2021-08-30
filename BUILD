package(default_visibility = ["//visibility:public"])

cc_library(
    name = "3_finger_gripper",
    srcs = [
        "gripper.cpp",
    ],
    hdrs = [
        "gripper.h"
    ]
)

cc_binary(
    name = "robotiq_api.so",
    linkshared = True,
    visibility = ["//visibility:public"],
    srcs = ["gripper.cpp", "gripper.h"],
)

cc_binary(
    name = "gripper_test",
    srcs = [
        "gripper_test.cpp",
    ],
    deps = [
        ":3_finger_gripper"
    ]
)