load("@rules_cc//cc:defs.bzl", "cc_library")

cc_test(
    name = "object_tracker_unit_test",
    srcs = ["object_tracker_unit_test.cpp"],
    visibility = ["//visibility:public"],
    deps = [
        "//object_tracking:object_tracking",
        "@com_google_googletest//:gtest_main",
    ],
)


cc_test(
    name = "transform_relative_to_origin",
    srcs = ["transform_relative_to_origin_test.cpp"],
    visibility = ["//visibility:public"],
    deps = [
        "@com_google_googletest//:gtest_main",
        "//object_tracking/utils:utils",
    ],
)

