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

Work in progress: the code lands through pull requests on `dev`.
