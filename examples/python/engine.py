"""Run an engine from a config file, command it through module ports and read its state.

    python examples/python/engine.py tests/fixtures/arm_loopback.yaml [--serve PORT]

With --serve the debug server listens on 127.0.0.1:PORT and the engine runs in real time until Ctrl-C.
"""
import sys

import kstone

engine = kstone.load_engine(sys.argv[1])
arm = engine.module("arm")
arm.write_port("target", [0.5, -0.3, 0.2])

if sys.argv[2:3] == ["--serve"]:
    server = engine.serve(port=int(sys.argv[3]))
    print(f"debug server on 127.0.0.1:{server.port}", flush=True)
    try:
        engine.run()
    except KeyboardInterrupt:
        pass
    finally:
        server.stop()
    sys.exit(0)

for _ in range(300):
    engine.step()
stats = engine.stats
print(f"t={engine.time:.2f}s cycles={stats['cycles']} failed={stats['failed_cycles']} quality={engine.result.quality}")
arm_q = [engine.q[engine.model.q_index(n)] for n in ("j1", "j2")]
ok = abs(arm_q[0] - 0.5) < 1e-3 and abs(arm_q[1] + 0.3) < 1e-3
print("reached" if ok else f"not reached: {arm_q}")
sys.exit(0 if ok else 1)
