#pragma once

#include <yaml-cpp/yaml.h>

#include <filesystem>

namespace larm {

// A configuration section handed to the module that owns it, e.g. a backend factory.
struct ConfigNode {
    YAML::Node node;
    // Relative paths inside the section resolve against this directory.
    std::filesystem::path baseDirectory;
};

} // namespace larm
