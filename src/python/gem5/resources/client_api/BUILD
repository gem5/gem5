load("@rules_python//python:defs.bzl", "py_library")

package(
    default_visibility = ["//visibility:public"],
)

py_library(
    name = "client_api",
    srcs = ["__init__.py"],
    deps = [
        ":abstract_client",
        ":azure_functions_client",
        ":client_query",
        ":jsonclient",
    ],
)

py_library(
    name = "abstract_client",
    srcs = ["abstract_client.py"],
)

py_library(
    name = "azure_functions_client",
    srcs = ["azure_functions_client.py"],
    deps = [
        ":abstract_client",
        ":client_query",
    ],
)

py_library(
    name = "client_query",
    srcs = ["client_query.py"],
)

py_library(
    name = "jsonclient",
    srcs = ["jsonclient.py"],
    deps = [
        ":abstract_client",
        ":client_query",
    ],
)
