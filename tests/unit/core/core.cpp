#include "kstone/core/clock.hpp"
#include "kstone/core/context.hpp"
#include "kstone/core/error.hpp"
#include "kstone/core/histogram.hpp"
#include "kstone/core/log.hpp"
#include "kstone/core/mask.hpp"
#include "kstone/core/resources.hpp"

#include <doctest/doctest.h>

#include <filesystem>
#include <string>
#include <vector>

namespace fs = std::filesystem;
using namespace kstone;

namespace {
Result<int> half(int x) {
    if (x % 2 != 0) return make_error(ErrorCode::invalid_argument, "odd");
    return x / 2;
}
}  // namespace

TEST_CASE("Result holds a value or an error") {
    auto ok = half(4);
    REQUIRE(ok);
    CHECK(*ok == 2);
    auto bad = half(3);
    REQUIRE_FALSE(bad);
    CHECK(bad.error().code == ErrorCode::invalid_argument);
    CHECK(bad.value_or(-1) == -1);
    CHECK_THROWS_AS(bad.value(), BadResultAccess);
    Status s = make_error(ErrorCode::io_error, "x");
    CHECK_FALSE(s.ok());
    CHECK(std::string(to_string(ErrorCode::not_found)) == "not_found");
}

TEST_CASE("Mask set algebra") {
    Mask a = Mask::from_indices(6, {0, 2, 4});
    Mask b = Mask(6).set_range(2, 3);
    CHECK(a.count() == 3);
    CHECK((a & b).indices() == std::vector<std::size_t>{2, 4});
    CHECK((a | b).indices() == std::vector<std::size_t>{0, 2, 3, 4});
    CHECK((a - b).indices() == std::vector<std::size_t>{0});
    CHECK((~a).indices() == std::vector<std::size_t>{1, 3, 5});
    CHECK_FALSE(Mask::none(3).any());
    CHECK(Mask::all(3) == ~Mask::none(3));
    CHECK_FALSE(a.test(99));
}

TEST_CASE("ManualClock is deterministic") {
    ManualClock clock(100);
    CHECK(clock.now_ns() == 100);
    clock.advance_ns(5);
    CHECK(clock.now_ns() == 105);
    clock.set_ns(7);
    CHECK(clock.now_ns() == 7);
    auto steady = make_steady_clock();
    const auto t0 = steady->now_ns();
    CHECK(steady->now_ns() >= t0);
}

TEST_CASE("Logger filters by level and stamps with its clock") {
    std::vector<std::string> seen;
    auto clock = std::make_shared<ManualClock>(42);
    Logger log(make_callback_sink([&](const LogRecord& r) {
                   CHECK(r.time_ns == 42);
                   seen.push_back(std::string(r.component) + ":" + std::string(r.message));
               }),
               LogLevel::info, clock);
    log.log(LogLevel::debug, "c", "hidden");
    log.log(LogLevel::warn, "c", "shown");
    CHECK(seen == std::vector<std::string>{"c:shown"});
    CHECK_FALSE(log.enabled(LogLevel::debug));
    log.set_level(LogLevel::off);
    CHECK_FALSE(log.enabled(LogLevel::error));
    CHECK_FALSE(Logger().enabled(LogLevel::error));
}

TEST_CASE("parse_uri") {
    CHECK(parse_uri("package://pkg/a.urdf").scheme == "package");
    CHECK(parse_uri("package://pkg/a.urdf").rest == "pkg/a.urdf");
    CHECK(parse_uri("C://x").scheme.empty());
    CHECK(parse_uri("relative/path").rest == "relative/path");
}

TEST_CASE("ResourceResolver resolves paths, search dirs and schemes") {
    const fs::path fixtures = KSTONE_FIXTURES;
    ResourceResolver r;
    CHECK(r.resolve((fixtures / "arm.urdf").string()));
    CHECK(r.resolve("arm.urdf", fixtures));
    CHECK(r.resolve("arm.urdf", fixtures / "nowhere").error().code == ErrorCode::not_found);
    CHECK(r.resolve("").error().code == ErrorCode::invalid_argument);
    CHECK(r.resolve("pkg://arm.urdf").error().code == ErrorCode::not_found);

    r.add_search_dir(fixtures);
    CHECK(r.resolve("arm.urdf", "/nonexistent"));

    r.add_scheme("pkg", make_directory_scheme({fixtures}));
    REQUIRE(r.has_scheme("pkg"));
    auto p = r.resolve("pkg://arm.urdf");
    REQUIRE(p);
    CHECK(fs::equivalent(*p, fixtures / "arm.urdf"));
    CHECK(r.resolve("pkg://../fixtures/arm.urdf").error().code == ErrorCode::invalid_argument);
    CHECK(r.resolve("pkg:///etc/passwd").error().code == ErrorCode::invalid_argument);

    r.add_scheme("boom", [](std::string_view) -> Result<fs::path> { throw std::runtime_error("x"); });
    CHECK(r.resolve("boom://a").error().code == ErrorCode::internal);

    auto text = r.read_text("pkg://arm.urdf");
    REQUIRE(text);
    CHECK(text->find("<robot") != std::string::npos);
    CHECK(read_text_file(fixtures / "missing").error().code == ErrorCode::io_error);
}

TEST_CASE("split_path_list") {
    const auto dirs = split_path_list("/a:/b::/c");
    REQUIRE(dirs.size() == 3);
    CHECK(dirs[2] == fs::path("/c"));
    CHECK(split_path_list("").empty());
}

TEST_CASE("Context::defaults wires the built-in loaders") {
    const Context ctx = Context::defaults();
    CHECK(ctx.clock);
    CHECK(ctx.log.enabled(LogLevel::warn));
    CHECK_FALSE(ctx.log.enabled(LogLevel::info));
    CHECK(ctx.loaders.find_format("URDF"));
    CHECK(ctx.loaders.find_extension(".XML"));
    CHECK(ctx.loaders.find_extension("mjcf"));
    CHECK_FALSE(ctx.loaders.find_extension("sdf"));
    CHECK(ctx.loaders.formats() == std::vector<std::string>{"urdf", "mjcf"});
}

TEST_CASE("LatencyHistogram buckets and quantiles") {
    using H = LatencyHistogram;
    CHECK(H::kBuckets == 162);
    CHECK(H::index(0) == 0);
    CHECK(H::index(1023) == 0);
    CHECK(H::index(1024) == 1);
    CHECK(H::index(1024 + 127) == 1);
    CHECK(H::index(1024 + 128) == 2);
    CHECK(H::index((std::int64_t{1} << 30) - 1) == H::kBuckets - 2);
    CHECK(H::index(std::int64_t{1} << 30) == H::kBuckets - 1);
    for (int i = 0; i < H::kBuckets - 1; ++i) {
        CHECK(H::index(H::upper_ns(i) - 1) == i);
        CHECK(H::index(H::upper_ns(i)) == i + 1);
    }
    // Relative bucket width stays within 12.5% across the range.
    for (std::int64_t ns = 1024; ns < (std::int64_t{1} << 30); ns = ns * 9 / 8 + 1) {
        const int i = H::index(ns);
        CHECK(static_cast<double>(H::upper_ns(i) - ns) <= 0.125 * static_cast<double>(ns) + 1.0);
    }

    H h;
    CHECK(h.quantile(0.5) == 0.0);
    for (int k = 1; k <= 1000; ++k) h.add(k * 1000);  // 1 us .. 1 ms
    h.add(-5);
    CHECK(h.count() == 1001);
    CHECK(h.max_ns() == 1000000);
    CHECK(h.quantile(0.5) == doctest::Approx(500e-6).epsilon(0.125));
    CHECK(h.quantile(0.99) == doctest::Approx(990e-6).epsilon(0.125));
    CHECK(h.quantile(1.0) == doctest::Approx(1e-3));
    CHECK(h.quantile(0.0) <= h.quantile(0.5));
    h.add(std::int64_t{5} << 30);
    CHECK(h.quantile(1.0) == doctest::Approx(5.0 * 1.073741824));
    h.clear();
    CHECK(h.count() == 0);
    CHECK(h.quantile(0.99) == 0.0);
}
