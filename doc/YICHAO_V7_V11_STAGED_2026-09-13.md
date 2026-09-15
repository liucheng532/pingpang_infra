# Yichao V7 model190 + Frozen CBF + V11 staged

The independent `yichao_v7_v11` deployment is implemented and staged on the
workstation at `/home/odl/codebase/yichao_v7_v11_adapter`. It is pinned to
Planner handoff `f052ebfc51239e83ff58c2067bbb30dd77292e59`, identity
`v7-model190-cbf-v11-i42500-f052ebf`, V7 model190 actor, Frozen V7 CBF, and
the existing hash-authenticated V11 i42500 Controller assets.

The implementation contains the exact 29D/87D feature builders, one-decision
per-shot latch, 51×51 CBF projection plus nominal/home candidates, 100:1
hitter priority, independent 0.50m hard guard, current-feedback X semantics,
Fixed relay/two-sided ACK/session safeguards, three deployment modes, and the
read-only Monitor with real/replay phase flows. The main Monitor decision card
shows only Rule-based command, final V7 command, and live position for the two
sides.

All model parity, ABI, deterministic replay, guard/state-machine, Monitor, and
latency checks passed across the local and workstation environments. Following
the shadow acceptance discovery, the V7 robot wrapper was strengthened to
require actual fresh samples on both torso topics before delegating to V11.
The suite is now 48/48 and the current 46-file payload manifest SHA256 is
`efdababeeb34b3f5aa1e87cdf19ecd757a30672e1ac911f924b16fc4a51e3af1`.

Both powered-on robots were reachable for a read-only asset audit. Their
frozen V11 and external assets matched, and neither Controller was running.
No robot file or process was changed.

The deployment remains stopped. ROS master retains a nonresponsive V6R10 node
registration on both command topics, so the V7 preflight correctly refuses to
start. The workstation-to-198 dedicated path `192.168.124.164` also remains
unreachable even though both management SSH addresses and robot-side `eth0`
links are online. This staging did not unregister the ROS node, change the
network, start Controller shadow, perform a transport-shadow ACK, enter
active, operate R2, or claim standing/hitting/return reliability. In
addition, model190 remains an update-190 experimental candidate without final
32-relay or physical reliability acceptance.

Detailed evidence is in
`doc/device_context/yichao_v7_v11_staging_20260913/README.md`.

Subsequent update at 20:10 CST: the user restored the 198 cable, both dedicated
links passed 3/3 pings, and there was no live old process to kill. Under the
user's cleanup request, the two exact dead V6R10 ROS publisher registrations
were unregistered. Three follow-up samples showed zero command publishers and
the complete V7 workstation `check shadow` passed. Nothing was started; see
`doc/device_context/yichao_v7_v11_cleanup_20260913_2010/README.md`.

Shadow acceptance was then attempted. It stopped fail-closed because ChingMu
provided neither the 198 torso rigid ID 0 nor ball rigid ID 30300. The 198 DDS
body state and ROS/TCPROS connectivity were present, but no left torso payload
existed. All workstation and robot processes were cleaned up normally; see
`doc/device_context/yichao_v7_v11_shadow_acceptance_20260913/README.md`.
