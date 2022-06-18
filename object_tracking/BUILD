load("@rules_cc//cc:defs.bzl", "cc_library")

cc_library(
    name = "object_tracking",
    srcs = [
    "object_tracker.cpp"],
    hdrs = [
    "object_tracker.h"],
    visibility = ["//visibility:public"],
    deps = [
        "//object_tracking/data_types:data_types",
        "//object_tracking/utils:utils",
    ],
)
