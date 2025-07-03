load("@npm//:defs.bzl", "npm_link_all_packages")
package(default_visibility = ["//:__subpackages__"])

alias(
    name = "format",
    actual = "//tools:format",
)

npm_link_all_packages(name = "node_modules")

