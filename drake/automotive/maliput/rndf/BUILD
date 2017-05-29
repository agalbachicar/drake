# -*- python -*-
# This file contains rules for Bazel; see drake/doc/bazel.rst.

load("//tools:cpplint.bzl", "cpplint")
load(
    "//tools:drake.bzl",
    "drake_cc_binary",
    "drake_cc_googletest",
    "drake_cc_library"
)

package(default_visibility = ["//visibility:public"])

drake_cc_library(
    name = "rndf",
    srcs = [],
    hdrs = [],
    deps = [
        ":loader",
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
        "spline_helpers.cc"
    ],
    hdrs = [
        "branch_point.h",
        "junction.h",
        "lane.h",
        "road_geometry.h",
        "segment.h",
        "spline_lane.h",
        "spline_helpers.h",
    ],
    deps = [
        "//drake/automotive/maliput/api",
        "//drake/common",
        "//drake/math:geometric_transform",
        "//drake/math:saturate",
        "@ignition_math",
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
        "@ignition_math",
        "@ignition_rndf",
    ],
)

drake_cc_library(
    name = "loader",
    srcs = ["loader.cc"],
    hdrs = ["loader.h"],
    deps = [
        "//drake/automotive/maliput/api",
        ":builder",
        "@ignition_math",
        "@ignition_rndf",
    ],
)

filegroup(
    name = "rndfs",
    srcs = glob(["maps/*.rndf"]),
)

# === test/ ===

drake_cc_googletest(
    name = "ignition_rndf_test",
    deps = [
        "@ignition_rndf",
    ],
)

drake_cc_googletest(
    name = "lane_test",
    size = "small",
    deps = [
        ":lanes",
    ],
)

drake_cc_googletest(
    name = "spline_lane_test",
    size = "small",
    deps = [
        ":lanes",
    ],
)

drake_cc_googletest(
    name = "spline_helper_test",
    size = "small",
    deps = [
        ":lanes",
    ],
)

drake_cc_googletest(
    name = "builder_test",
    size = "small",
    deps = [
        ":builder",
    ],
)

drake_cc_googletest(
    name = "segment_test",
    size = "small",
    deps = [
        ":lanes",
    ],
)

drake_cc_binary(
    name = "rndf_load",
    testonly = 1,
    srcs = ["test/rndf_load.cc"],
    deps = [
        ":loader",
        "//drake/common:text_logging_gflags",
    ],
)

py_test(
    name = "rndf_load_test",
    size = "small",
    srcs = ["test/rndf_load_test.py"],
    args = ["drake/automotive/maliput/rndf/rndf_load"],
    data = [
        ":rndf_load",
        ":rndfs",
    ],
)

cpplint()
