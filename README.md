# estop-core

The RAMMP robot-wide emergency stop as one ESP-IDF component. Add it to a firmware, give it
a GPIO and your RTPS participant, and the board takes part in the e-stop on `rammp/estop`.

- **`estop::Latch`**: the board that owns the stop (the MIB). Starts stopped, stops on any
  e-stop press or a silent board, and ends the stop when the clear button is held.
- **`estop::Node`**: any board with an e-stop switch (the HMI, others). A press stops the
  robot until the latch clears it.

## Use it in your ESP-IDF project

**1. Add the dependency** in `main/idf_component.yml`:

```yaml
dependencies:
  estop_core:
    git: https://github.com/Alaining/estop-core.git
    version: v0.2.0
```

It brings espp/rtps (1.3.10 or newer), espp/cdr and the RAMMP message (`rammp_rtps_messages`). If your
firmware already has rammp-rtps as a submodule, point the message at it so there is one copy:

```yaml
  rammp_rtps_messages:
    override_path: "../external/rammp-rtps/components/rammp_rtps_messages"
```

**2. Create one object, static,** and start it first in `app_main`:

```cpp
#include "estop/estop.hpp"

// The MIB: watches the HMI, clear button on GPIO35 (held 2 s).
static estop::Latch latch({.clear_input = {.gpio_num = GPIO_NUM_35}});

// ...or a board with a switch: the HMI, switch on GPIO2.
// static estop::Node node({.name = "hmi", .input = {.gpio_num = GPIO_NUM_2}});

extern "C" void app_main() {
  latch.start();                       // first: stopped until cleared
  // ... bring up the network, create and start your espp::RtpsParticipant ...
  latch.attach(participant);           // adds the rammp/estop writer and reader
  // ...
  if (latch.active()) { stop_motors(); }  // every cycle, any task
}
```

**3. Wire the switch.** The default input is a normally-open switch from the GPIO to GND,
with the internal pull-up (pressed = LOW). Change it per input:

```cpp
.input = {.gpio_num = GPIO_NUM_2,
          .active_level = estop::ActiveLevel::LOW,  // HIGH for a normally-closed switch
          .pull = estop::Pull::UP}                  // or DOWN, or NONE (external resistor)
```

**4. Give the RTPS tasks a high enough priority.** The latch has a **deadman**: it treats
a watched board it has not heard from for 300 ms as dead, and stops the robot. That is what
catches a crashed board or a pulled cable. But the deadman can't tell a dead board from one
that is alive but too busy to send. Every e-stop message goes out and comes in through
espp's RTPS tasks (`rtps_worker_<n>`, `rtps_reactor`), which run at priority 5 by default. If
a task of higher priority keeps the CPU for more than 300 ms (an LVGL screen build at 20,
for example), the e-stop messages wait, and the deadman stops the robot for nothing.

So set the RTPS tasks at or above your busiest task, in `sdkconfig.defaults`:

```
CONFIG_RTPS_THREAD_POOL_READER_PRIO=20
```

(menuconfig: Component config → RTPS → Transport task priority (advanced); espp 1.3.10 or
newer, which estop-core requires). These tasks also run every RTPS callback in your app, so
keep the callbacks short and never block in them, for example on a UI lock: a blocked
callback holds up the e-stop messages the same way.

That's all. The complete, commented programs are `examples/latch` (MIB) and
`examples/node` (HMI).

## API

| Call | `Latch` (MIB) | `Node` (board with a switch) |
| --- | --- | --- |
| `start()` | Arms the inputs, starts the task. Once, first in app_main. | Same. |
| `attach(participant)` | Adds the e-stop writer and reader. Once, after start(). | Same. |
| `active()` | The stop is in force: no motion. | Do not drive: pressed, or the latch says stopped. |
| `engage(reason)` | Stop for your own reason (a fault). | Same as a press. |
| `start_clear_attempt()` | End the stop, as holding the clear button does. | |
| `set_input(input)` | | Move the switch (e.g. from a setting). |
| `status()` | A copy for the UI and the log: cause, attempts, releases. | A copy: counters, the latch's reason. |

`active()` is one atomic load, safe from any task. Until `start()` succeeds it is true.

## How it works

- Every board sends its state every 100 ms. A press sends ENGAGED at once and keeps sending
  it, switch released or not, until the latch clears it.
- The latch stops on any ENGAGED, and on a watched board silent for 300 ms (deadman).
- A clear attempt asks every board to clear; a board refuses while its switch is held. The
  latch releases once all have answered, or gives up after 1 s.
- The interrupt only counts the press: no polling, nothing allocated after `start()`.

The message and the timing are in the RAMMP RTPS spec
([rammp-rtps](https://github.com/rammp-org/rammp-rtps), `messages/estop_message.hpp`).

## Repository

| Path | What |
| --- | --- |
| `include/estop/estop.hpp` | `estop::Latch`, `estop::Node`, `estop::Input`. |
| `include/estop/core/` | The rules in plain C++ (no I/O, no clock): codec, node, latch. |
| `src/esp/` | The tasks, the interrupt, RTPS. |
| `examples/` | Complete programs for the ESP32-P4. |
| `test/` | Host tests, including the node-against-latch race suite. |

Host tests: `cmake -B build && cmake --build build -j && ctest --test-dir build`
(GCC 14+, ASan/UBSan on).
