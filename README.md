# estop-core

The RAMMP robot-wide emergency stop, as one ESP-IDF component: everything a firmware needs
to take part in the e-stop on the `rammp/estop` RTPS topic, behind a small API.

- `estop::Node`: any node that can request the stop (the HMI, other boards). It takes the
  switch GPIO, handles the edge interrupt, sends its state every 100 ms, and accepts the
  latch's clear attempts.
- `estop::Latch`: the node that owns the latch (the MIB). It latches on any request or on a
  silent node (deadman), makes clear attempts, and releases the stop.

The message and its timing come from the RAMMP RTPS spec
([rammp-rtps](https://github.com/rammp-org/rammp-rtps), `messages/estop_message.hpp`).

## Layout

| Path | What |
| --- | --- |
| `include/estop/core/codec.hpp` | Encode (espp/cdr) and an allocation-free decode of `rammp::EStop`. |
| `include/estop/core/node_logic.hpp` | A node's rules (the HMI's): heartbeat, press, clear-attempt acceptance. No I/O, no clock. |
| `include/estop/core/latch_logic.hpp` | The latch's rules (the MIB's): latch, deadman, clear attempt, release. No I/O, no clock. |
| `test/` | Host tests: codec, node, latch, and the end-to-end race suite (node against latch through the codec). |
| `test/idf_app/` | Builds the component for the ESP32-P4 through the component manager. |

The ESP-IDF glue (`estop::Node`, `estop::Latch`: GPIO interrupt, task, RTPS writer and
reader) comes next, on top of the core.

## Use from a firmware

```yaml
# main/idf_component.yml
dependencies:
  estop_core:
    git: https://github.com/Alaining/estop-core.git
    version: <commit or tag>
```

The component pulls `rammp_rtps_messages` from rammp-rtps; a firmware with rammp-rtps as a
submodule (in `EXTRA_COMPONENT_DIRS`) uses its own copy instead.

## Host tests

```sh
cmake -B build && cmake --build build -j && ctest --test-dir build --output-on-failure
```

C++23 (GCC 14 or newer), ASan and UBSan on. The build fetches rammp-rtps and cdr; to use
local checkouts, add `-DFETCHCONTENT_SOURCE_DIR_RAMMP_RTPS=<path>` or
`-DFETCHCONTENT_SOURCE_DIR_CDR=<path>`.
