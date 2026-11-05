#include "kstone/module/yaml.hpp"

#include "kstone/module/registry.hpp"

#include <doctest/doctest.h>

#include <cstdlib>
#include <filesystem>
#include <limits>
#include <map>
#include <string>

using namespace kstone;
namespace fs = std::filesystem;

namespace {
const fs::path kFixtures = KSTONE_FIXTURES;

ConfigNode parse(const std::string& text) {
    auto r = parse_yaml(text, "t.yaml");
    REQUIRE_MESSAGE(r, (r ? "" : r.error().message));
    return *r;
}

std::string parse_error(const std::string& text) {
    auto r = parse_yaml(text, "t.yaml");
    REQUIRE_FALSE(r);
    CHECK(r.error().code == ErrorCode::parse_error);
    return r.error().message;
}

ModelPtr load_arm() {
    auto m = Model::load(ModelSource::file((kFixtures / "arm.urdf").string()), {}, Context::defaults());
    REQUIRE_MESSAGE(m, (m ? "" : m.error().message));
    return *m;
}

std::string populate_error(const std::string& text, ErrorCode code) {
    auto set = ModuleSet::create(load_arm());
    REQUIRE(set);
    const ConfigNode doc = parse(text);
    Status s = ModuleRegistry::with_builtins().populate(*set, doc, Context::defaults());
    REQUIRE_FALSE(s);
    CHECK(s.error().code == code);
    return s.error().message;
}

const char* kFrameTask = R"(
  type: frame_task
  frame: tool
  trajectory:
    translation: {velocity: 0.3, acceleration: 1, jerk: 10}
    rotation: {velocity: 1, acceleration: 1, jerk: 10}
)";
}  // namespace

TEST_CASE("yaml: scalar typing") {
    const ConfigNode n = parse(R"(
a: 1
b: -2.5e1
c: true
d: False
e: ~
f:
g: "1"
h: '3.0'
i: hello world
j: [1, two, null]
k: {x: 1}
l: 0x10
m: yes
)");
    REQUIRE(n.kind() == ConfigNode::Kind::map);
    CHECK(n.find("a")->as_number() == 1.0);
    CHECK(n.find("b")->as_number() == -25.0);
    CHECK(n.find("c")->as_bool());
    CHECK_FALSE(n.find("d")->as_bool());
    CHECK(n.find("e")->is_null());
    CHECK(n.find("f")->is_null());
    CHECK(n.find("g")->as_string() == "1");
    CHECK(n.find("h")->as_string() == "3.0");
    CHECK(n.find("i")->as_string() == "hello world");
    const ConfigNode& j = *n.find("j");
    REQUIRE(j.size() == 3);
    CHECK(j.items()[0].as_number() == 1.0);
    CHECK(j.items()[1].as_string() == "two");
    CHECK(j.items()[2].is_null());
    CHECK(n.find("k")->find("x")->as_number() == 1.0);
    // Non-decimal forms and YAML 1.1 yes/no stay strings; the schema reports them instead of coercing.
    CHECK(n.find("l")->as_string() == "0x10");
    CHECK(n.find("m")->as_string() == "yes");
}

TEST_CASE("yaml: infinity spellings") {
    const ConfigNode n = parse("[.inf, -.inf, Infinity, +Infinity, -Infinity, \"Infinity\"]");
    REQUIRE(n.size() == 6);
    const double inf = std::numeric_limits<double>::infinity();
    CHECK(n.items()[0].as_number() == inf);
    CHECK(n.items()[1].as_number() == -inf);
    CHECK(n.items()[2].as_number() == inf);
    CHECK(n.items()[3].as_number() == inf);
    CHECK(n.items()[4].as_number() == -inf);
    CHECK(n.items()[5].as_string() == "Infinity");
}

TEST_CASE("yaml: locations") {
    const ConfigNode n = parse("a:\n  b: 2\n  c:\n");
    CHECK(n.find("a")->location().line == 2);
    const ConfigNode& b = *n.find("a")->find("b");
    CHECK(b.location().file == "t.yaml");
    CHECK(b.location().line == 2);
    CHECK(b.location().column == 6);
    CHECK(n.find("a")->find("c")->location().line == 3);
    CHECK(parse("").is_null());
}

TEST_CASE("yaml: malformed documents") {
    CHECK(parse_error("a: [1, 2\n").find("t.yaml:") == 0);
    CHECK(parse_error("a: 1\na: 2\n").find("t.yaml:2:1: duplicate key 'a'") == 0);
    CHECK(parse_error("? [1]\n: 2\n").find("map keys must be scalars") != std::string::npos);
    CHECK(parse_error("a: !custom 1\n").find("unsupported tag") != std::string::npos);
    CHECK(parse_error("a:\n  <<: {b: 1}\n").find("merge keys ('<<') are not supported") != std::string::npos);
    std::string deep(300, '[');
    CHECK(parse_error(deep + std::string(300, ']')).find("nested too deeply") != std::string::npos);
    std::string deep_map;
    for (int i = 0; i < 5000; ++i) deep_map += "{\"a\":";
    CHECK(parse_error(deep_map + "1" + std::string(5000, '}')).find("nested too deeply") !=
          std::string::npos);
}

TEST_CASE("yaml: load_yaml reports missing files") {
    auto r = load_yaml(kFixtures / "does_not_exist.yaml");
    REQUIRE_FALSE(r);
    CHECK(r.error().code == ErrorCode::io_error);
}

TEST_CASE("yaml: diagnostics carry KST codes") {
    CHECK(parse_error("a: [1, 2\n").find("[KST-E001]") != std::string::npos);
    CHECK(parse_error("a: 1\na: 2\n").find("[KST-E002]") != std::string::npos);
    CHECK(parse_error("? [1]\n: 2\n").find("[KST-E003]") != std::string::npos);
    CHECK(parse_error("a: !custom 1\n").find("[KST-E004]") != std::string::npos);
    CHECK(parse_error(std::string(300, '[') + std::string(300, ']')).find("[KST-E005]") != std::string::npos);
    // Without options, include tags and ${...} are not interpreted.
    CHECK(parse_error("a: !include x.yaml\n").find("unsupported tag '!include'") != std::string::npos);
    CHECK(parse("a: ${HOME}\n").find("a")->as_string() == "${HOME}");
    CHECK(to_string(ConfigDiag::model_failed) == "KST-E301");
}

namespace {
YamlOptions vars(std::map<std::string, std::string> v) {
    YamlOptions o;
    o.expand_vars = true;
    o.vars = std::move(v);
    return o;
}

Result<ConfigNode> parse_with(const std::string& text, const YamlOptions& o) {
    return parse_yaml(text, "t.yaml", o);
}
}  // namespace

TEST_CASE("yaml: variables") {
    const YamlOptions o = vars({{"N", "3"}, {"FLAG", "true"}, {"NAME", "arm"}});
    auto r = parse_with(R"(
n: ${N}
flag: ${FLAG}
quoted: "${N}"
partial: x${N}
two: ${N}${N}
name: ${NAME}
dflt: ${MISSING:-1.5}
empty: ${MISSING:-}
escaped: $${N}
plain: no vars
)",
                        o);
    REQUIRE_MESSAGE(r, (r ? "" : r.error().message));
    CHECK(r->find("n")->as_number() == 3.0);
    CHECK(r->find("flag")->as_bool());
    CHECK(r->find("quoted")->as_string() == "3");
    CHECK(r->find("partial")->as_string() == "x3");
    CHECK(r->find("two")->as_string() == "33");
    CHECK(r->find("name")->as_string() == "arm");
    CHECK(r->find("dflt")->as_number() == 1.5);
    CHECK(r->find("empty")->is_null());
    CHECK(r->find("escaped")->as_string() == "${N}");
    CHECK(r->find("plain")->as_string() == "no vars");
    CHECK(r->find("n")->location().line == 2);

    auto u = parse_with("a: 1\nb: ${NOPE}\n", o);
    REQUIRE_FALSE(u);
    CHECK(u.error().code == ErrorCode::not_found);
    CHECK_MESSAGE(u.error().message.find("t.yaml:2:4: undefined variable 'NOPE' [KST-E008]") == 0,
                  u.error().message);
    for (const char* bad : {"a: ${N\n", "a: ${}\n", "a: ${1x}\n", "a: ${a b}\n", "a: ${X:-${N}}\n"}) {
        auto b = parse_with(bad, o);
        REQUIRE_FALSE(b);
        CHECK_MESSAGE(b.error().message.find("[KST-E009]") != std::string::npos, b.error().message);
    }
    auto empty = parse_with("a: ${E:-7}\nb: ${E}\n", vars({{"E", ""}}));
    REQUIRE_MESSAGE(empty, (empty ? "" : empty.error().message));
    CHECK(empty->find("a")->as_number() == 7.0);
    CHECK(empty->find("b")->is_null());
    auto unused = parse_with("n: ${N}\nflag: ${FLAG}\n", o);
    REQUIRE_FALSE(unused);
    CHECK(unused.error().code == ErrorCode::invalid_argument);
    CHECK_MESSAGE(
        unused.error().message.find("variable 'NAME' is set but never referenced") != std::string::npos,
        unused.error().message);
    CHECK(unused.error().message.find("[KST-E010]") != std::string::npos);
}

TEST_CASE("yaml: unquoted reference in a flow map hints at quoting") {
    const YamlOptions o = vars({{"F", "x"}});
    auto r = parse_with("m: {file: ${F}}\n", o);
    REQUIRE_FALSE(r);
    CHECK_MESSAGE(r.error().message.find("quote ${...}") != std::string::npos, r.error().message);
    CHECK(parse_with("m: {file: \"${F}\"}\n", o)->find("m")->find("file")->as_string() == "x");
    auto plain = parse_with("m: {file: x\n", o);
    REQUIRE_FALSE(plain);
    CHECK(plain.error().message.find("quote") == std::string::npos);
}

TEST_CASE("yaml: environment variables are opt-in") {
    YamlOptions o = vars({});
    CHECK_FALSE(parse_with("a: ${PATH}\n", o));
    o.environment = true;
    auto r = parse_with("a: ${PATH}\n", o);
    REQUIRE(r);
    CHECK(r->find("a")->as_string() == std::getenv("PATH"));
    o.vars["PATH"] = "override";
    CHECK(parse_with("a: ${PATH}\n", o)->find("a")->as_string() == "override");
}

TEST_CASE("yaml: includes") {
    const YamlOptions files = YamlOptions::files();
    auto r = load_yaml(kFixtures / "arm_include.yaml", files);
    REQUIRE_MESSAGE(r, (r ? "" : r.error().message));
    CHECK(r->find("rate_hz")->as_number() == 100.0);
    const ConfigNode& tool = *r->find("modules")->find("tool");
    CHECK(tool.find("gain")->as_number() == 0.5);
    const ConfigNode& translation = *tool.find("trajectory")->find("translation");
    CHECK(translation.find("jerk")->as_number() == 10.0);
    // Locations inside included files name those files.
    CHECK(translation.location().file == (kFixtures / "include" / "trajectory.yaml").string());
    CHECK(translation.location().line == 1);

    YamlOptions set = files;
    set.vars = {{"RATE", "250"}};
    CHECK(load_yaml(kFixtures / "arm_include.yaml", set)->find("rate_hz")->as_number() == 250.0);

    // Disabled includes keep the old strict behaviour.
    CHECK_FALSE(load_yaml(kFixtures / "arm_include.yaml"));

    YamlOptions text = files;
    text.base_dir = kFixtures / "include";
    auto t = parse_yaml("traj: !include trajectory.yaml\n", "t.yaml", text);
    REQUIRE_MESSAGE(t, (t ? "" : t.error().message));
    CHECK(t->find("traj")->find("rotation")->find("jerk")->as_number() == 20.0);
    // The included root itself is located at the !include, not at 1:1 of the other file.
    CHECK(t->find("traj")->location().file == "t.yaml");
    CHECK(t->find("traj")->location().column == 7);
    CHECK(t->find("traj")->find("rotation")->location().file ==
          (kFixtures / "include" / "trajectory.yaml").string());
}

TEST_CASE("yaml: include failures") {
    YamlOptions o = YamlOptions::files();
    o.base_dir = kFixtures / "include";
    const auto error = [&](const std::string& text) {
        auto r = parse_yaml(text, "t.yaml", o);
        REQUIRE_FALSE(r);
        return r.error();
    };
    Error e = error("x: 1\ny: !include missing.yaml\n");
    CHECK(e.code == ErrorCode::io_error);
    CHECK_MESSAGE(e.message.find("t.yaml:2:4: cannot include 'missing.yaml'") == 0, e.message);
    CHECK(e.message.find("[KST-E006]") != std::string::npos);

    e = error("y: !include cycle_a.yaml\n");
    CHECK_MESSAGE(e.message.find((kFixtures / "include" / "cycle_b.yaml").string() +
                                 ":2:7: include cycle through") == 0,
                  e.message);
    CHECK(e.message.find("[KST-E007]") != std::string::npos);

    e = error("y: !include broken.yaml\n");
    CHECK_MESSAGE(e.message.find((kFixtures / "include" / "broken.yaml").string() + ":") == 0, e.message);
    CHECK(e.message.find("[KST-E001]") != std::string::npos);

    e = error("y: !include [a.yaml]\n");
    CHECK(e.message.find("[KST-E004]") != std::string::npos);

    o.max_include_depth = 0;
    e = error("y: !include trajectory.yaml\n");
    CHECK(e.message.find("[KST-E005]") != std::string::npos);
}

TEST_CASE("populate: builds modules in reference order") {
    auto set = ModuleSet::create(load_arm());
    REQUIRE(set);
    // ns is declared before the frame task it references.
    const ConfigNode doc = parse(std::string(R"(
ns: {type: null_space_preference, primary: tool, preference: manipulability}
arm: {type: joint_target, joints: [j1, j2, j3]}
tool:)") + kFrameTask);
    Status s = ModuleRegistry::with_builtins().populate(*set, doc, Context::defaults());
    REQUIRE_MESSAGE(s, (s ? "" : s.error().message));
    CHECK(set->modules().size() == 3);
    CHECK(set->module("ns"));
    CHECK(std::string(set->module("tool")->type()) == "frame_task");
    CHECK(set->configure());
    CHECK(ModuleRegistry::with_builtins().populate(*set, ConfigNode{}, Context::defaults()));
}

TEST_CASE("populate: errors carry location and path") {
    std::string m = populate_error("arm: {type: joint_targett, joints: [j1]}\n", ErrorCode::not_found);
    CHECK_MESSAGE(m.find("t.yaml:1:13: modules.arm.type: unknown module type 'joint_targett' (known: ") == 0,
                  m);

    m = populate_error("arm: {joints: [j1]}\n", ErrorCode::invalid_argument);
    CHECK_MESSAGE(m.find("modules.arm: missing 'type'") != std::string::npos, m);

    m = populate_error("arm:\n  type: joint_target\n  joints: [j1]\n  wieght: 2\n",
                       ErrorCode::invalid_argument);
    CHECK_MESSAGE(m.find("modules.arm.wieght: unknown key") != std::string::npos, m);

    m = populate_error("ns: {type: null_space_preference, primary: tool}\n", ErrorCode::not_found);
    CHECK_MESSAGE(m.find("modules.ns.primary: no module 'tool'") != std::string::npos, m);

    m = populate_error("- 1\n", ErrorCode::invalid_argument);
    CHECK_MESSAGE(m.find("expected a map of module ids") != std::string::npos, m);

    m = populate_error(R"(
a: {type: null_space_preference, primary: b}
b: {type: null_space_preference, primary: a}
)",
                       ErrorCode::invalid_argument);
    CHECK_MESSAGE(m.find("reference cycle among modules 'a', 'b'") != std::string::npos, m);
}

TEST_CASE("to_json: keeps valid UTF-8 and replaces malformed bytes") {
    CHECK(to_json(ConfigNode(std::string("\xe4\xb8\xad \xf0\x9f\x98\x80"))) ==
          "\"\xe4\xb8\xad \xf0\x9f\x98\x80\"");
    CHECK(to_json(ConfigNode(std::string("a\xff\xfe"
                                         "b"))) == "\"a\\ufffd\\ufffdb\"");
    CHECK(to_json(ConfigNode(std::string("\xe4\xb8"))) == "\"\\ufffd\\ufffd\"");
    CHECK(to_json(ConfigNode(std::string("\xed\xa0\x80"))) == "\"\\ufffd\\ufffd\\ufffd\"");
    CHECK(to_json(ConfigNode(std::string("\xc0\xaf"))) == "\"\\ufffd\\ufffd\"");
}

TEST_CASE("yaml: rejects content after the document") {
    for (const char* text : {"{a: 1} garbage", "{a: 1}\nb: 2", "- a\nb: 1", "\"s\" t", "a: 1\n---\nb: 2"}) {
        CAPTURE(text);
        CHECK(parse_error(text).find("unexpected content after the end of the document") !=
              std::string::npos);
    }
    CHECK(parse("a: 1\n...\n").find("a")->as_number() == 1.0);
    CHECK(parse("a: 1\n---\n").find("a")->as_number() == 1.0);
    CHECK(parse("").kind() == ConfigNode::Kind::null);
}
