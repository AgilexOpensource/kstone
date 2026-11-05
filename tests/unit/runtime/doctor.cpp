// Environment self-check: smoke checks pass here, library classification and conflict detection.
#include "kstone/runtime/doctor.hpp"

#include "kstone/core/version.hpp"

#include <doctest/doctest.h>

#include <algorithm>

using namespace kstone;

TEST_CASE("doctor passes in the test environment") {
    const DoctorReport report = doctor();
    for (const auto& c : report.checks) {
        INFO(c.name << ": " << c.detail);
        CHECK(c.ok);
    }
    CHECK(report.ok());
    CHECK(report.version == library_version_string());
    CHECK(report.checks.size() == 3);
#ifdef __linux__
    CHECK(std::any_of(report.libraries.begin(), report.libraries.end(),
                      [](const LoadedLibrary& l) { return l.name.rfind("libpinocchio", 0) == 0; }));
#endif
    const ConfigNode node = to_config_node(report);
    REQUIRE(node.find("ok") != nullptr);
    CHECK(node.find("ok")->as_bool());
    CHECK(node.find("checks")->size() == 3);
    CHECK(to_text(report).find("ok    solve") != std::string::npos);
}

TEST_CASE("doctor classifies library providers") {
    CHECK(library_provider("/x/.venv/lib/python3.10/site-packages/cmeel.prefix/lib/libcoal.so") == "pypi");
    CHECK(library_provider("/opt/ros/humble/lib/x86_64-linux-gnu/libpinocchio.so.2.6.21") == "ros");
    CHECK(library_provider("/usr/lib/x86_64-linux-gnu/libboost_system.so.1.74.0") == "system");
    CHECK(library_provider("/home/u/build/libkstone.so") == "other");
}

TEST_CASE("doctor flags duplicate and mixed libraries") {
    const LoadedLibrary pin_pypi{"libpinocchio_default.so",
                                 "/v/site-packages/cmeel.prefix/lib/libpinocchio_default.so", "pypi"};
    const LoadedLibrary coal_pypi{"libcoal.so", "/v/site-packages/cmeel.prefix/lib/libcoal.so", "pypi"};
    const LoadedLibrary coal_ros{"libcoal.so.3.0.1", "/opt/ros/jazzy/lib/libcoal.so.3.0.1", "ros"};
    const LoadedLibrary boost_sys{"libboost_filesystem.so.1.74.0", "/usr/lib/libboost_filesystem.so.1.74.0",
                                  "system"};

    CHECK(check_libraries({pin_pypi, coal_pypi, boost_sys}).ok);
    CHECK(check_libraries({}).ok);

    const DoctorCheck mixed = check_libraries({pin_pypi, coal_ros});
    CHECK_FALSE(mixed.ok);
    CHECK(mixed.detail.find("mixed providers") != std::string::npos);

    const DoctorCheck twice = check_libraries({pin_pypi, coal_pypi, coal_ros});
    CHECK_FALSE(twice.ok);
    CHECK(twice.detail.find("libcoal.so loaded twice") != std::string::npos);
}
