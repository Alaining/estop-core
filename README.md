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

## Use

```cpp
#include "estop/estop.hpp"

// A node (the HMI): its switch on GPIO2, normally open to GND.
static estop::Node node({.name = "hmi", .gpio = 2, .contact = estop::Contact::NO});

// The latch (the MIB): watches "hmi", clear button on GPIO35 held for 2 s.
static estop::Latch latch({.clear_gpio = 35});

extern "C" void app_main() {
  node.start();               // first: the switch works from here, before the network
  // ... bring the network up, create and start the espp::RtpsParticipant ...
  node.attach(participant);   // adds the rammp/estop writer and reader
  if (node.active()) { /* do not drive */ }
}
```

| Call | Node | Latch |
| --- | --- | --- |
| `start()` | Arms the switch, starts the task. Once, from app_main. | Same, plus the clear input. |
| `attach(participant)` | Adds the e-stop writer and reader. Once, after start(). | Same. |
| `active()` | Do not drive: engaged, latch says latched, switch not armed, not started. One atomic load, any task or ISR. | Latched, or not started. |
| `engage(reason)` | Requests the stop as a press does. Any task. | Latches with this node's reason. |
| `set_input(gpio, contact)` | Moves the switch (from a setting). | |
| `start_clear_attempt()` | | The operator asks to end the stop (as holding the clear input does). |
| `status()` | A copy of the state (counters, latch's reason) for the UI and the log. | Same (cause, attempts, releases). |

The objects hold their task's stack and queues, so declare them static. After `start()`
nothing allocates, except `attach()` (the endpoints) and, on the latch, causes longer than
15 characters (room reserved in `start()`). The switch reaches the task through an edge
interrupt that only counts: a press shorter than any poll period still counts. The
GPIO ISR service is installed if no one has.

`examples/node` and `examples/latch` are complete programs (network through IDF's
`protocol_examples_common`, the P4's internal Ethernet).

## Layout

| Path | What |
| --- | --- |
| `include/estop/estop.hpp` | `estop::Node`, `estop::Latch`: the ESP-IDF API. |
| `include/estop/core/` | The rules, plain C++ with no I/O and no clock: `codec.hpp` (encode through espp/cdr, allocation-free decode), `node_logic.hpp`, `latch_logic.hpp`. |
| `src/esp/` | The tasks, the edge interrupt, RTPS. |
| `test/` | Host tests: codec, node, latch, and the end-to-end race suite (node against latch through the codec). |
| `test/idf_app/`, `examples/` | Built for the ESP32-P4 in CI through the component manager. |

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
