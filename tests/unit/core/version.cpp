#include "kstone/core/version.hpp"

#include <doctest/doctest.h>

#include <string>

TEST_CASE("library version matches the headers it was built with") {
    const auto v = kstone::library_version();
    CHECK(v.major == KSTONE_VERSION_MAJOR);
    CHECK(v.minor == KSTONE_VERSION_MINOR);
    CHECK(v.patch == KSTONE_VERSION_PATCH);
    CHECK(std::string(kstone::library_version_string()) == KSTONE_VERSION_STRING);
}
