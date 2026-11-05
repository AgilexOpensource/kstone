#include "cli.hpp"
#include <csignal>

int main(int argc, char** argv) {
    std::signal(SIGPIPE,
                SIG_IGN);  // Output to a closed pipe must not kill the process before the robot is stopped.
    return kstone_cli_main(argc, argv);
}
