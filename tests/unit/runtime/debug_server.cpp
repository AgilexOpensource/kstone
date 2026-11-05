#include "kstone/runtime/debug_server.hpp"

#include "kstone/module/yaml.hpp"
#include "kstone/runtime/config.hpp"
#include "kstone/runtime/engine.hpp"

#include <doctest/doctest.h>

#include <algorithm>
#include <arpa/inet.h>
#include <atomic>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <memory>
#include <netinet/in.h>
#include <stdexcept>
#include <string>
#include <sys/socket.h>
#include <sys/time.h>
#include <thread>
#include <unistd.h>
#include <vector>

using namespace kstone;
namespace fs = std::filesystem;

namespace {
const fs::path kFixtures = KSTONE_FIXTURES;

class Client {
  public:
    explicit Client(std::uint16_t port) {
        fd_ = ::socket(AF_INET, SOCK_STREAM, 0);
        REQUIRE(fd_ >= 0);
        timeval tv{5, 0};
        ::setsockopt(fd_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof tv);
        sockaddr_in sa{};
        sa.sin_family = AF_INET;
        sa.sin_port = htons(port);
        sa.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
        REQUIRE(::connect(fd_, reinterpret_cast<const sockaddr*>(&sa), sizeof sa) == 0);
    }
    ~Client() { ::close(fd_); }
    Client(const Client&) = delete;
    Client& operator=(const Client&) = delete;

    ConfigNode read() {
        unsigned char h[4];
        REQUIRE(read_exact(h, 4));
        const std::uint32_t len = (std::uint32_t{h[0]} << 24) | (std::uint32_t{h[1]} << 16) |
                                  (std::uint32_t{h[2]} << 8) | std::uint32_t{h[3]};
        std::string text(len, '\0');
        REQUIRE(read_exact(text.data(), len));
        auto doc = parse_yaml(text, "frame");
        REQUIRE(doc);
        return std::move(*doc);
    }

    ConfigNode read_type(const std::string& type) {
        for (;;) {
            ConfigNode doc = read();
            if (doc.find("type")->as_string() == type) return doc;
        }
    }

    void send(const std::string& text) {
        const auto n = static_cast<std::uint32_t>(text.size());
        std::string f{static_cast<char>(n >> 24), static_cast<char>((n >> 16) & 0xff),
                      static_cast<char>((n >> 8) & 0xff), static_cast<char>(n & 0xff)};
        f += text;
        REQUIRE(::send(fd_, f.data(), f.size(), MSG_NOSIGNAL) == static_cast<ssize_t>(f.size()));
    }

    bool read_exact(void* out, std::size_t n) {
        auto* p = static_cast<char*>(out);
        while (n > 0) {
            const ssize_t r = ::recv(fd_, p, n, 0);
            if (r <= 0) return false;
            p += r;
            n -= static_cast<std::size_t>(r);
        }
        return true;
    }

  private:
    int fd_{-1};
};

ConfigNode reply_to(Client& c, int id) {
    for (;;) {
        ConfigNode r = c.read_type("reply");
        const ConfigNode* i = r.find("id");
        if (i != nullptr && i->kind() == ConfigNode::Kind::number && static_cast<int>(i->as_number()) == id)
            return r;
    }
}

std::string code_of(const ConfigNode& reply) {
    const ConfigNode* c = reply.find("code");
    return c != nullptr ? c->as_string() : std::string{};
}

const ConfigNode* module_node(const ConfigNode& doc, const std::string& id) {
    for (const ConfigNode& m : doc.find("modules")->items())
        if (m.find("id")->as_string() == id) return &m;
    return nullptr;
}
std::string base64_decode(const std::string& in) {
    static const std::string table = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    std::string out;
    std::uint32_t v = 0;
    int bits = 0;
    for (char ch : in) {
        const auto pos = table.find(ch);
        if (pos == std::string::npos) break;
        v = (v << 6) | static_cast<std::uint32_t>(pos);
        bits += 6;
        if (bits >= 8) {
            bits -= 8;
            out += static_cast<char>((v >> bits) & 0xff);
        }
    }
    return out;
}

void write_file(const fs::path& path, const std::string& text) {
    fs::create_directories(path.parent_path());
    std::ofstream(path, std::ios::binary) << text;
}

std::string fetch(Client& c, const std::string& name, int id) {
    std::string data;
    for (;;) {
        c.send(R"({"op":"asset","id":)" + std::to_string(id) + R"(,"name":")" + name + R"(","offset":)" +
               std::to_string(data.size()) + "}");
        const ConfigNode r = reply_to(c, id);
        REQUIRE_MESSAGE(r.find("ok")->as_bool(), code_of(r));
        data += base64_decode(r.find("data")->as_string());
        if (data.size() >= static_cast<std::size_t>(r.find("total")->as_number())) return data;
    }
}
}  // namespace

TEST_CASE("debug server: hello, state, commands and replies over loopback") {
    auto e = load_engine(kFixtures / "arm_loopback.yaml");
    REQUIRE(e);
    DebugServerOptions options;
    options.port = 0;
    options.publish_hz = 200.0;
    options.urdf = kFixtures / "arm.urdf";
    auto server = DebugServer::start(*e, options);
    REQUIRE_MESSAGE(server, (server ? std::string{} : server.error().message));
    REQUIRE((*server)->port() != 0);

    Client client((*server)->port());
    ConfigNode hello = client.read();
    CHECK(hello.find("type")->as_string() == "hello");
    CHECK(hello.find("version")->as_number() == 1);
    CHECK_FALSE(hello.find("auth_required")->as_bool());
    CHECK(hello.find("extensions") == nullptr);
    {
        std::vector<std::string> caps;
        for (const ConfigNode& n : hello.find("capabilities")->items()) caps.push_back(n.as_string());
        for (const char* op :
             {"ping", "subscribe", "acquire", "release", "set_port", "tune", "pause", "step", "asset"})
            CHECK_MESSAGE(std::find(caps.begin(), caps.end(), op) != caps.end(), op);
    }
    ConfigNode model = client.read();
    CHECK(model.find("type")->as_string() == "model");
    CHECK(model.find("modules")->items().size() == 3);
    const auto& joints = model.find("joints")->items();
    REQUIRE(joints.size() >= 2);
    CHECK(joints.at(0).find("parent")->as_string() == "universe");
    CHECK(joints.at(1).find("parent")->as_string() == joints.at(0).find("name")->as_string());
    {
        const Model& m = *e->modules().model();
        for (const ConfigNode& jn : joints) {
            const std::string name = jn.find("name")->as_string();
            const JointInfo& j = m.joints()[*m.joint_id(name)];
            CAPTURE(name);
            CHECK(jn.find("idx_v")->as_number() == j.idx_v);
            CHECK(jn.find("nv")->as_number() == j.nv);
            const ConfigNode* lim = jn.find("limits");
            REQUIRE(lim);
            CHECK(lim->find("position")->as_bool() == m.has_position_limit(j.id));
            REQUIRE(lim->find("lower")->items().size() == static_cast<std::size_t>(j.nq));
            REQUIRE(lim->find("velocity")->items().size() == static_cast<std::size_t>(j.nv));
            for (Index i = 0; i < j.nq; ++i) {
                CHECK(lim->find("lower")->items()[static_cast<std::size_t>(i)].as_number() ==
                      m.lower_position_limit()[j.idx_q + i]);
                CHECK(lim->find("upper")->items()[static_cast<std::size_t>(i)].as_number() ==
                      m.upper_position_limit()[j.idx_q + i]);
            }
            for (Index i = 0; i < j.nv; ++i) {
                CHECK(lim->find("velocity")->items()[static_cast<std::size_t>(i)].as_number() ==
                      m.velocity_limit()[j.idx_v + i]);
                CHECK(lim->find("effort")->items()[static_cast<std::size_t>(i)].as_number() ==
                      m.effort_limit()[j.idx_v + i]);
            }
        }
        const auto& mimics = model.find("mimics")->items();
        REQUIRE(mimics.size() == 1);
        CHECK(mimics[0].find("follower")->as_string() == "finger_right_joint");
        CHECK(mimics[0].find("leader")->as_string() == "finger_left_joint");
        CHECK(mimics[0].find("multiplier")->as_number() == -1.0);
        CHECK(mimics[0].find("offset")->as_number() == 0.0);
    }
    const ConfigNode* arm = module_node(model, "arm");
    REQUIRE(arm);
    const ConfigNode& port = arm->find("ports")->items().at(0);
    CHECK(port.find("name")->as_string() == "target");
    CHECK(port.find("labels")->items().size() == 3);

    std::atomic<bool> stop{false};
    std::thread runner([&] { CHECK(e->run(0.0, &stop)); });

    {
        const ConfigNode st = client.read_type("state");
        for (const char* key : {"cycle_p50", "cycle_p99", "cycle_p999"}) {
            const ConfigNode* n = st.find("stats")->find(key);
            REQUIRE_MESSAGE(n, key);
            CHECK(n->as_number() >= 0.0);
        }
    }
    client.send(R"({"op":"set_port","id":1,"module":"arm","port":"target","value":[0.5,-0.3,1.0]})");
    client.send(R"({"op":"home","id":2,"module":"nope"})");
    client.send(R"({"op":"set_port","id":3,"module":"arm","port":"target","value":[1]})");
    client.send(R"({"op":"set_enabled","id":4,"module":"fingers","enabled":false})");
    client.send("not json");

    bool ok[5]{};
    bool seen[5]{};
    for (int n = 0; n < 5; ++n) {
        ConfigNode reply = client.read_type("reply");
        const ConfigNode* id = reply.find("id");
        const int i =
            id != nullptr && id->kind() == ConfigNode::Kind::number ? static_cast<int>(id->as_number()) : 0;
        REQUIRE(i >= 0);
        REQUIRE(i < 5);
        seen[i] = true;
        ok[i] = reply.find("ok")->as_bool();
        if (!ok[i]) CHECK(reply.find("error") != nullptr);
    }
    CHECK((seen[0] && seen[1] && seen[2] && seen[3] && seen[4]));
    CHECK(ok[1]);
    CHECK_FALSE(ok[2]);
    CHECK_FALSE(ok[3]);
    CHECK(ok[4]);
    CHECK_FALSE(ok[0]);

    // Wait for enough simulated time to converge.
    double start_time = -1.0;
    for (;;) {
        ConfigNode state = client.read_type("state");
        const double t = state.find("time")->as_number();
        if (start_time < 0.0) start_time = t;
        if (t - start_time < 25.0) continue;
        CHECK_FALSE(module_node(state, "fingers")->find("enabled")->as_bool());
        const ConfigNode* preempted = module_node(state, "arm")->find("preempted");
        REQUIRE(preempted != nullptr);
        CHECK(preempted->kind() == ConfigNode::Kind::number);
        const auto& target = module_node(state, "arm")->find("ports")->find("target")->items();
        REQUIRE(target.size() == 3);
        CHECK(target[0].as_number() == doctest::Approx(0.5));
        CHECK(state.find("stats")->find("cycles")->as_number() > 0);
        bool owned = false;
        for (const ConfigNode& r : state.find("residuals")->items()) {
            const ConfigNode* owner = r.find("module");
            REQUIRE(owner != nullptr);
            if (owner->kind() != ConfigNode::Kind::string || owner->as_string() != "arm") continue;
            CHECK(r.find("name")->as_string() == "arm/" + r.find("task")->as_string());
            owned = true;
        }
        CHECK(owned);
        const auto& stages = state.find("stages")->items();
        REQUIRE_FALSE(stages.empty());
        CHECK(stages[0].find("status")->kind() == ConfigNode::Kind::string);
        CHECK(stages[0].find("iterations")->kind() == ConfigNode::Kind::number);
        CHECK(stages[0].find("held")->kind() == ConfigNode::Kind::boolean);
        CHECK(stages[0].find("solve_s")->kind() == ConfigNode::Kind::number);
        const std::string quality = state.find("quality")->as_string();
        CHECK((quality == "nominal" || quality == "degraded"));
        break;
    }
    stop = true;
    runner.join();

    const Model& m = *e->modules().model();
    CHECK(*m.joint_position(e->state().q, *m.joint_id("j1")) == doctest::Approx(0.5).epsilon(1e-3));
    CHECK(*m.joint_position(e->state().q, *m.joint_id("j2")) == doctest::Approx(-0.3).epsilon(1e-3));
    (*server)->stop();
    (*server)->stop();
}

TEST_CASE("debug server: rejects non-loopback hosts and bad rates") {
    auto e = load_engine(kFixtures / "arm_loopback.yaml");
    REQUIRE(e);
    DebugServerOptions options;
    options.port = 0;
    options.host = "0.0.0.0";
    CHECK_FALSE(DebugServer::start(*e, options));
    options.token = "short";
    CHECK_FALSE(DebugServer::start(*e, options));
    options.token.clear();
    options.host = "localhost";
    options.command_timeout_s = 0.0;
    CHECK_FALSE(DebugServer::start(*e, options));
    options.command_timeout_s = 1.0;
    options.publish_hz = 0.0;
    CHECK_FALSE(DebugServer::start(*e, options));
    options.publish_hz = 30.0;
    auto server = DebugServer::start(*e, options);
    REQUIRE(server);
    CHECK((*server)->port() != 0);
}

TEST_CASE("debug server: token auth") {
    auto e = load_engine(kFixtures / "arm_loopback.yaml");
    REQUIRE(e);
    DebugServerOptions options;
    options.port = 0;
    options.host = "0.0.0.0";  // Allowed once a token is set.
    options.token = "0123456789abcdef-secret";
    auto server = DebugServer::start(*e, options);
    REQUIRE_MESSAGE(server, (server ? std::string{} : server.error().message));

    {
        Client c((*server)->port());
        ConfigNode hello = c.read();
        CHECK(hello.find("auth_required")->as_bool());
        c.send(R"({"op":"ping","id":1})");
        CHECK(code_of(c.read()) == "unauthorized");
        char byte = 0;
        CHECK_FALSE(c.read_exact(&byte, 1));  // Closed.
    }
    {
        Client c((*server)->port());
        c.read();
        c.send(R"({"op":"auth","id":1,"token":"0123456789abcdef-wrong!"})");
        CHECK(code_of(c.read()) == "unauthorized");
        char byte = 0;
        CHECK_FALSE(c.read_exact(&byte, 1));
    }
    Client c((*server)->port());
    c.read();
    c.send(R"({"op":"auth","id":1,"token":"0123456789abcdef-secret"})");
    CHECK(c.read().find("ok")->as_bool());
    CHECK(c.read().find("type")->as_string() == "model");
    CHECK(c.read().find("type")->as_string() == "lease");
    c.send(R"({"op":"ping","id":2})");
    CHECK(reply_to(c, 2).find("ok")->as_bool());
    // A wrong token after auth is reported but keeps the session.
    c.send(R"({"op":"auth","id":3,"token":"0123456789abcdef-wrong!"})");
    CHECK(code_of(reply_to(c, 3)) == "unauthorized");
    c.send(R"({"op":"ping","id":4})");
    CHECK(reply_to(c, 4).find("ok")->as_bool());
}

TEST_CASE("debug server: lease, subscribe and command timeout") {
    auto e = load_engine(kFixtures / "arm_loopback.yaml");
    REQUIRE(e);
    DebugServerOptions options;
    options.port = 0;
    options.publish_hz = 100.0;
    auto server = DebugServer::start(*e, options);
    REQUIRE(server);

    Client a((*server)->port());
    Client b((*server)->port());
    a.read_type("lease");
    b.read_type("lease");

    // Engine is not stepping: queued commands time out.
    a.send(R"({"op":"home","id":1,"module":"arm","timeout_s":0.05})");
    CHECK(code_of(reply_to(a, 1)) == "timeout");
    a.send(R"({"op":"home","id":2,"module":"arm","timeout_s":-1})");
    CHECK(code_of(reply_to(a, 2)) == "invalid_argument");
    a.send(R"({"op":"bogus","id":2})");
    {
        const ConfigNode r = reply_to(a, 2);
        CHECK(code_of(r) == "unsupported");
        CHECK(r.find("error")->as_string() == "unknown op 'bogus'");
    }

    a.send(R"({"op":"acquire","id":3})");
    CHECK(reply_to(a, 3).find("ok")->as_bool());
    ConfigNode lease = b.read_type("lease");
    CHECK_FALSE(lease.find("mine")->as_bool());
    CHECK(lease.find("holder")->kind() == ConfigNode::Kind::number);
    b.send(R"({"op":"set_enabled","id":4,"module":"fingers","enabled":false})");
    CHECK(code_of(reply_to(b, 4)) == "lease_held");
    b.send(R"({"op":"acquire","id":5})");
    CHECK(code_of(reply_to(b, 5)) == "lease_held");
    b.send(R"({"op":"subscribe","id":6,"rate_hz":1000})");
    CHECK(code_of(reply_to(b, 6)) == "invalid_argument");
    b.send(R"({"op":"subscribe","id":7,"rate_hz":0})");
    CHECK(reply_to(b, 7).find("ok")->as_bool());

    std::atomic<bool> stop{false};
    std::thread runner([&] { CHECK(e->run(0.0, &stop)); });
    a.send(R"({"op":"set_enabled","id":8,"module":"fingers","enabled":false})");
    CHECK(reply_to(a, 8).find("ok")->as_bool());
    a.read_type("state");

    // Holder releases: b may acquire; b is unsubscribed so it sees no state.
    a.send(R"({"op":"release","id":9})");
    CHECK(reply_to(a, 9).find("ok")->as_bool());
    lease = b.read_type("lease");
    CHECK(lease.find("holder")->kind() == ConfigNode::Kind::null);
    b.send(R"({"op":"acquire","id":10})");
    for (;;) {
        ConfigNode doc = b.read();
        const std::string type = doc.find("type")->as_string();
        REQUIRE(type != "state");
        if (type == "reply") {
            CHECK(doc.find("ok")->as_bool());
            break;
        }
    }
    CHECK(b.read_type("lease").find("mine")->as_bool());
    a.send(R"({"op":"acquire","id":11,"force":true})");
    CHECK(reply_to(a, 11).find("ok")->as_bool());
    CHECK_FALSE(b.read_type("lease").find("mine")->as_bool());
    stop = true;
    runner.join();
}

TEST_CASE("debug server: capabilities without assets") {
    auto e = load_engine(kFixtures / "arm_loopback.yaml");
    REQUIRE(e);
    DebugServerOptions options;
    options.port = 0;
    options.serve_assets = false;
    auto server = DebugServer::start(*e, options);
    REQUIRE(server);
    Client a((*server)->port());
    ConfigNode hello = a.read_type("hello");
    for (const ConfigNode& n : hello.find("capabilities")->items()) CHECK(n.as_string() != "asset");
    a.read_type("lease");
    a.send(R"({"op":"x-demo.echo","id":1})");
    CHECK(code_of(reply_to(a, 1)) == "unsupported");
    (*server)->stop();
}

TEST_CASE("debug server: a connection beyond the client limit is told busy") {
    auto e = load_engine(kFixtures / "arm_loopback.yaml");
    REQUIRE(e);
    DebugServerOptions options;
    options.port = 0;
    auto server = DebugServer::start(*e, options);
    REQUIRE(server);
    std::vector<std::unique_ptr<Client>> held;
    for (int i = 0; i < 8; ++i) {
        held.push_back(std::make_unique<Client>((*server)->port()));
        held.back()->read_type("hello");
    }
    Client extra((*server)->port());
    ConfigNode first = extra.read();
    CHECK(first.find("type")->as_string() == "reply");
    CHECK(code_of(first) == "busy");
    char byte = 0;
    CHECK_FALSE(extra.read_exact(&byte, 1));
    held.pop_back();
    for (int attempt = 0;; ++attempt) {
        Client again((*server)->port());
        if (again.read().find("type")->as_string() == "hello") break;
        REQUIRE(attempt < 50);
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    (*server)->stop();
}

TEST_CASE("debug server: tune op and model params") {
    auto e = load_engine(kFixtures / "arm_loopback.yaml");
    REQUIRE(e);
    DebugServerOptions options;
    options.port = 0;
    options.urdf = kFixtures / "arm.urdf";
    auto server = DebugServer::start(*e, options);
    REQUIRE(server);

    Client a((*server)->port());
    Client b((*server)->port());
    const ConfigNode model = a.read_type("model");
    b.read_type("model");
    const ConfigNode* arm = module_node(model, "arm");
    REQUIRE(arm);
    CHECK(arm->find("homeable")->as_bool());
    CHECK(arm->find("dependencies")->kind() == ConfigNode::Kind::list);
    const auto& driven = arm->find("driven")->items();
    REQUIRE(driven.size() == 3);
    CHECK(driven[0].as_string() == "j1");
    CHECK(driven[2].as_string() == "j3");
    // joint_target defaults to level 0.
    const auto& levels = arm->find("levels")->items();
    REQUIRE(levels.size() == 1);
    CHECK(levels[0].as_number() == 0.0);
    CHECK(arm->find("tunable")->as_bool());
    CHECK(arm->find("params")->find("gain") != nullptr);
    CHECK(arm->find("tune_schema")->kind() == ConfigNode::Kind::map);

    std::atomic<bool> stop{false};
    std::thread runner([&] { CHECK(e->run(0.0, &stop)); });

    a.send(R"({"op":"tune","id":1,"module":"arm","params":{"gain":0.25}})");
    CHECK(reply_to(a, 1).find("ok")->as_bool());
    for (Client* c : {&a, &b}) {
        const ConfigNode p = c->read_type("params");
        CHECK(p.find("module")->as_string() == "arm");
        CHECK(p.find("values")->find("gain")->as_number() == doctest::Approx(0.25));
    }
    a.send(R"({"op":"tune","id":2,"module":"arm","params":{"gain":-1}})");
    const ConfigNode bad = reply_to(a, 2);
    CHECK_FALSE(bad.find("ok")->as_bool());
    CHECK(bad.find("error")->as_string().find("104") != std::string::npos);
    a.send(R"({"op":"tune","id":3,"module":"arm","params":[1]})");
    CHECK(code_of(reply_to(a, 3)) == "invalid_argument");
    a.send(R"({"op":"tune","id":4,"module":"arm","params":{"joints":["j1"]}})");
    CHECK_FALSE(reply_to(a, 4).find("ok")->as_bool());

    // A late client gets the tuned values right after the start-time model.
    Client late((*server)->port());
    CHECK(module_node(late.read_type("model"), "arm")->find("params")->find("gain")->as_number() !=
          doctest::Approx(0.25));
    const ConfigNode replay = late.read_type("params");
    CHECK(replay.find("module")->as_string() == "arm");
    CHECK(replay.find("values")->find("gain")->as_number() == doctest::Approx(0.25));

    stop = true;
    runner.join();
    (*server)->stop();
}

TEST_CASE("debug server: tune_solver op and model solver") {
    auto e = load_engine(kFixtures / "arm_loopback.yaml");
    REQUIRE(e);
    DebugServerOptions options;
    options.port = 0;
    auto server = DebugServer::start(*e, options);
    REQUIRE(server);

    Client a((*server)->port());
    Client b((*server)->port());
    const ConfigNode model = a.read_type("model");
    b.read_type("model");
    const ConfigNode* solver = model.find("solver");
    REQUIRE(solver);
    CHECK(solver->find("params")->find("damping") != nullptr);
    CHECK(solver->find("tune_schema")->find("properties")->find("mode") != nullptr);

    std::atomic<bool> stop{false};
    std::thread runner([&] { CHECK(e->run(0.0, &stop)); });

    a.send(R"({"op":"tune_solver","id":1,"params":{"limit_gain":0.3}})");
    CHECK(reply_to(a, 1).find("ok")->as_bool());
    for (Client* c : {&a, &b})
        CHECK(c->read_type("solver_params").find("values")->find("limit_gain")->as_number() ==
              doctest::Approx(0.3));
    a.send(R"({"op":"tune_solver","id":2,"params":{"limit_gain":2}})");
    CHECK_FALSE(reply_to(a, 2).find("ok")->as_bool());
    a.send(R"({"op":"tune_solver","id":3})");
    CHECK(code_of(reply_to(a, 3)) == "invalid_argument");

    // A held lease blocks other clients.
    a.send(R"({"op":"acquire","id":4})");
    CHECK(reply_to(a, 4).find("ok")->as_bool());
    b.send(R"({"op":"tune_solver","id":5,"params":{"limit_gain":0.4}})");
    CHECK_FALSE(reply_to(b, 5).find("ok")->as_bool());

    Client late((*server)->port());
    late.read_type("model");
    CHECK(late.read_type("solver_params").find("values")->find("limit_gain")->as_number() ==
          doctest::Approx(0.3));

    stop = true;
    runner.join();
    (*server)->stop();
}

TEST_CASE("debug server: tune_limits op, broadcast and replay") {
    auto e = load_engine(kFixtures / "arm_loopback.yaml");
    REQUIRE(e);
    DebugServerOptions options;
    options.port = 0;
    auto server = DebugServer::start(*e, options);
    REQUIRE(server);

    Client a((*server)->port());
    const ConfigNode model = a.read_type("model");
    REQUIRE(model.find("joint_limits"));
    CHECK(model.find("joint_limits")->find("velocity")->entries().empty());

    std::atomic<bool> stop{false};
    std::thread runner([&] { CHECK(e->run(0.0, &stop)); });

    a.send(R"({"op":"tune_limits","id":1,"params":{"velocity":{"j1":0.5}}})");
    CHECK(reply_to(a, 1).find("ok")->as_bool());
    CHECK(a.read_type("joint_limits").find("values")->find("velocity")->find("j1")->as_number() ==
          doctest::Approx(0.5));
    a.send(R"({"op":"tune_limits","id":2,"params":{"velocity":{"j1":99}}})");
    CHECK(code_of(reply_to(a, 2)) == "invalid_argument");
    a.send(R"({"op":"tune_limits","id":3})");
    CHECK(code_of(reply_to(a, 3)) == "invalid_argument");

    Client late((*server)->port());
    late.read_type("model");
    CHECK(late.read_type("joint_limits").find("values")->find("velocity")->find("j1")->as_number() ==
          doctest::Approx(0.5));

    stop = true;
    runner.join();
    (*server)->stop();
}

TEST_CASE("debug server: set_rate op and state rate_hz") {
    auto e = load_engine(kFixtures / "arm_loopback.yaml");
    REQUIRE(e);
    const double initial = e->options().rate_hz;
    DebugServerOptions options;
    options.port = 0;
    auto server = DebugServer::start(*e, options);
    REQUIRE(server);
    Client a((*server)->port());
    a.read_type("model");

    std::atomic<bool> stop{false};
    std::thread runner([&] { CHECK(e->run(0.0, &stop)); });
    CHECK(a.read_type("state").find("rate_hz")->as_number() == doctest::Approx(initial));
    a.send(R"({"op":"set_rate","id":1,"rate_hz":50})");
    CHECK(reply_to(a, 1).find("ok")->as_bool());
    bool seen = false;
    for (int i = 0; i < 50 && !seen; ++i) seen = a.read_type("state").find("rate_hz")->as_number() == 50.0;
    CHECK(seen);
    a.send(R"({"op":"set_rate","id":2,"rate_hz":0})");
    CHECK(code_of(reply_to(a, 2)) == "invalid_argument");
    a.send(R"({"op":"set_rate","id":3})");
    CHECK(code_of(reply_to(a, 3)) == "invalid_argument");
    stop = true;
    runner.join();
    (*server)->stop();
    CHECK(e->options().rate_hz == 50.0);
}

TEST_CASE("debug server: serves the URDF and the files it references") {
    const fs::path dir = fs::temp_directory_path() / ("kstone-assets-" + std::to_string(::getpid()));
    fs::remove_all(dir);
    std::ifstream src(kFixtures / "arm.urdf");
    std::string urdf((std::istreambuf_iterator<char>(src)), std::istreambuf_iterator<char>());
    const std::string link = "<link name=\"link1\">";
    REQUIRE(urdf.find(link) != std::string::npos);
    urdf.replace(urdf.find(link), link.size(),
                 link + R"(<visual><geometry><mesh filename="meshes/part.obj"/></geometry></visual>)"
                        R"(<visual><geometry><mesh filename="meshes/gone.stl"/></geometry></visual>)");
    write_file(dir / "robot" / "arm.urdf", urdf);
    write_file(dir / "robot" / "meshes" / "part.obj", "mtllib part.mtl\nv 0 0 0\n");
    write_file(dir / "robot" / "meshes" / "part.mtl", "newmtl m\nmap_Kd tex/skin.png\n");
    write_file(dir / "robot" / "meshes" / "tex" / "skin.png", "abc");
    std::ifstream cfg_in(kFixtures / "arm_loopback.yaml");
    std::string cfg((std::istreambuf_iterator<char>(cfg_in)), std::istreambuf_iterator<char>());
    write_file(dir / "robot" / "engine.yaml", cfg);

    auto e = load_engine(dir / "robot" / "engine.yaml");
    REQUIRE_MESSAGE(e, (e ? std::string{} : e.error().message));
    DebugServerOptions options;
    options.port = 0;
    options.token = "0123456789abcdef-secret";
    auto server = DebugServer::start(*e, options);
    REQUIRE(server);

    Client stranger((*server)->port());
    stranger.read_type("hello");
    stranger.send(R"({"op":"asset","id":1,"name":"model.urdf"})");
    CHECK(code_of(reply_to(stranger, 1)) == "unauthorized");

    Client c((*server)->port());
    c.read_type("hello");
    c.send(R"({"op":"auth","id":1,"token":"0123456789abcdef-secret"})");
    const ConfigNode model = c.read_type("model");
    CHECK(fs::path(model.find("urdf")->as_string()) == fs::weakly_canonical(dir / "robot" / "arm.urdf"));
    const ConfigNode* assets = model.find("assets");
    REQUIRE(assets);
    REQUIRE(assets->kind() == ConfigNode::Kind::map);
    CHECK(assets->find("id")->as_string().size() == 32);
    CHECK(assets->find("model")->as_string() == "model.urdf");
    const ConfigNode* files = assets->find("files");
    for (const char* name :
         {"model.urdf", "assets/000/part.obj", "assets/000/part.mtl", "assets/000/tex/skin.png"})
        CHECK_MESSAGE(files->find(name) != nullptr, name);
    CHECK(files->size() == 4);
    const ConfigNode* png = files->find("assets/000/tex/skin.png");
    REQUIRE(png);
    CHECK(png->find("sha256")->as_string() ==
          "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad");
    CHECK(png->find("size")->as_number() == 3);
    REQUIRE(assets->find("missing")->items().size() == 1);
    CHECK(assets->find("missing")->items().at(0).as_string() == "meshes/gone.stl");

    const std::string doc = fetch(c, "model.urdf", 2);
    CHECK(doc.size() == static_cast<std::size_t>(files->find("model.urdf")->find("size")->as_number()));
    CHECK(doc.find(R"(filename="assets/000/part.obj")") != std::string::npos);
    CHECK(doc.find(R"(filename="meshes/gone.stl")") != std::string::npos);
    CHECK(fetch(c, "assets/000/tex/skin.png", 3) == "abc");
    CHECK(fetch(c, "assets/000/part.obj", 4) == "mtllib part.mtl\nv 0 0 0\n");

    c.send(R"({"op":"asset","id":5,"name":"../arm.urdf"})");
    CHECK(code_of(reply_to(c, 5)) == "not_found");
    c.send(R"({"op":"asset","id":6,"name":"model.urdf","offset":1e9})");
    CHECK(code_of(reply_to(c, 6)) == "out_of_range");
    c.send(R"({"op":"asset","id":7,"name":"model.urdf","offset":-1})");
    CHECK(code_of(reply_to(c, 7)) == "invalid_argument");
    (*server)->stop();

    DebugServerOptions off = options;
    off.serve_assets = false;
    auto quiet = DebugServer::start(*e, off);
    REQUIRE(quiet);
    Client d((*quiet)->port());
    d.read_type("hello");
    d.send(R"({"op":"auth","id":1,"token":"0123456789abcdef-secret"})");
    CHECK(d.read_type("model").find("assets")->kind() == ConfigNode::Kind::null);
    d.send(R"({"op":"asset","id":2,"name":"model.urdf"})");
    CHECK(code_of(reply_to(d, 2)) == "not_found");
    (*quiet)->stop();
    fs::remove_all(dir);
}

TEST_CASE("debug server: pause and step ops") {
    auto e = load_engine(kFixtures / "arm_loopback.yaml");
    REQUIRE(e);
    DebugServerOptions options;
    options.port = 0;
    options.publish_hz = 200.0;
    auto server = DebugServer::start(*e, options);
    REQUIRE_MESSAGE(server, (server ? std::string{} : server.error().message));
    Client client((*server)->port());
    client.read_type("model");

    std::atomic<bool> stop{false};
    std::thread runner([&] { CHECK(e->run(0.0, &stop)); });

    {
        const ConfigNode st = client.read_type("state");
        CHECK_FALSE(st.find("paused")->as_bool());
        CHECK(st.find("fresh")->as_bool());
        CHECK(st.find("stats")->find("held_cycles")->as_number() == 0);
        CHECK(st.find("q_command")->items().size() == st.find("q")->items().size());
    }
    client.send(R"({"op":"step","id":1})");
    CHECK(code_of(reply_to(client, 1)) == "invalid_argument");
    client.send(R"({"op":"pause","id":2})");
    CHECK(code_of(reply_to(client, 2)) == "invalid_argument");
    client.send(R"({"op":"pause","id":3,"paused":true})");
    CHECK(reply_to(client, 3).find("ok")->as_bool());
    client.send(R"({"op":"step","id":4,"cycles":0})");
    CHECK(code_of(reply_to(client, 4)) == "invalid_argument");
    client.send(R"({"op":"step","id":5,"cycles":1.5})");
    CHECK(code_of(reply_to(client, 5)) == "invalid_argument");
    client.send(R"({"op":"step","id":6,"cycles":3})");
    CHECK(reply_to(client, 6).find("ok")->as_bool());

    for (;;) {
        const ConfigNode st = client.read_type("state");
        if (!st.find("paused")->as_bool() || st.find("stats")->find("held_cycles")->as_number() < 10)
            continue;
        break;
    }
    client.send(R"({"op":"pause","id":7,"paused":false})");
    CHECK(reply_to(client, 7).find("ok")->as_bool());
    for (;;) {
        const ConfigNode st = client.read_type("state");
        if (st.find("paused")->as_bool()) continue;
        break;
    }
    stop = true;
    runner.join();
    CHECK_FALSE(e->paused());
    CHECK(e->stats().held_cycles >= 10);
    (*server)->stop();
}

TEST_CASE("debug server: module visuals and display outputs") {
    const fs::path dir = fs::temp_directory_path() / ("kstone-display-" + std::to_string(::getpid()));
    fs::remove_all(dir);
    // Two sibling bodies, so their pair survives the adjacency filter.
    write_file(dir / "pair.urdf", R"(<robot name="pair">
  <link name="base"/>
  <link name="slider"><collision><geometry><sphere radius="0.1"/></geometry></collision></link>
  <link name="post"><collision><geometry><sphere radius="0.1"/></geometry></collision></link>
  <joint name="slide" type="prismatic">
    <parent link="base"/><child link="slider"/><axis xyz="1 0 0"/>
    <limit lower="-0.5" upper="0.6" velocity="2.0" effort="100"/>
  </joint>
  <joint name="spin" type="revolute">
    <origin xyz="0.5 0 0"/><parent link="base"/><child link="post"/><axis xyz="0 0 1"/>
    <limit lower="-1" upper="1" velocity="1" effort="1"/>
  </joint>
</robot>)");
    auto doc = parse_yaml(
        "model: {file: pair.urdf}\nrate_hz: 100\nrobot: {type: loopback, initial: [0.265, 0]}\n"
        "modules:\n  collision: {type: self_collision, urdf: " +
            (dir / "pair.urdf").string() +
            "}\n"
            "  world: {type: world_collision, urdf: " +
            (dir / "pair.urdf").string() +
            ", obstacles: [{name: ball, shape: {type: sphere, size: [0.1, 0, 0]}, enabled: false}]}\n",
        "inline");
    REQUIRE(doc);
    auto e = build_engine(*doc, dir);
    REQUIRE_MESSAGE(e, (e ? std::string{} : e.error().message));
    DebugServerOptions options;
    options.port = 0;
    auto server = DebugServer::start(*e, options);
    REQUIRE_MESSAGE(server, (server ? std::string{} : server.error().message));
    Client client((*server)->port());
    const ConfigNode model = client.read_type("model");
    const ConfigNode* collision = module_node(model, "collision");
    REQUIRE(collision);
    const auto& visuals = collision->find("visuals")->items();
    REQUIRE(visuals.size() == 2);
    CHECK(visuals[0].find("shape")->find("type")->as_string() == "sphere");
    CHECK(visuals[0].find("shape")->find("size")->items()[0].as_number() == doctest::Approx(0.1));
    const auto& outputs = collision->find("outputs")->items();
    REQUIRE(outputs.size() == 7);
    CHECK(outputs[4].find("type")->as_string() == "text");
    CHECK(outputs[3].find("name")->as_string() == "band");
    CHECK(outputs[3].find("type")->as_string() == "visuals");
    CHECK(outputs[3].find("role")->as_string() == "warning");
    CHECK(outputs[0].find("role")->as_string() == "margin");
    CHECK(outputs[0].find("unit")->as_string() == "m");
    CHECK(outputs[1].find("unit") == nullptr);
    bool has_p99 = false;
    for (const ConfigNode& stat : model.find("stats")->items())
        if (stat.find("name")->as_string() == "cycle_p99")
            has_p99 = stat.find("kind")->as_string() == "duration" && stat.find("unit")->as_string() == "s";
    CHECK(has_p99);

    REQUIRE(e->step(0.0));
    const ConfigNode st = client.read_type("state");
    const ConfigNode* display = module_node(st, "collision")->find("display");
    REQUIRE(display);
    REQUIRE(display->kind() == ConfigNode::Kind::map);
    CHECK(display->find("at")->as_string() == "measured");
    const ConfigNode* values = display->find("outputs");
    CHECK(values->find("min_distance")->as_number() == doctest::Approx(0.035).epsilon(1e-3));
    CHECK(values->find("band")->items().size() == 2);
    CHECK(values->find("closest")->items().size() == 2);
    CHECK(values->find("pair")->kind() == ConfigNode::Kind::string);
    CHECK_FALSE(values->find("pair")->as_string().empty());
    CHECK(values->find("pairs")->as_number() == 1.0);
    const auto& poses = display->find("visual_poses")->items();
    REQUIRE(poses.size() == 2);
    double xs = 0.0;
    for (const ConfigNode& p : poses) xs += p.find("translation")->items()[0].as_number();
    CHECK(xs == doctest::Approx(0.765).epsilon(1e-3));
    // world_collision draws only its obstacles; a disabled one is hidden (null).
    const auto& world_poses = module_node(st, "world")->find("display")->find("visual_poses")->items();
    REQUIRE(world_poses.size() == 1);
    CHECK(world_poses[0].kind() == ConfigNode::Kind::null);
    fs::remove_all(dir);
}
