#pragma once

/**
 * @file estop.hpp
 * @brief The RAMMP robot-wide e-stop on an ESP32: estop::Node for a node that can
 *        request the stop (the HMI, other boards), estop::Latch for the node that owns
 *        the latch (rammp::kEStopClearer, the MIB).
 *
 * Declare the object static (it holds its task's stack and queues), start() it from
 * app_main, and attach() it to the firmware's RTPS participant once that exists. One
 * task per object owns the rules (estop/core: NodeLogic, LatchLogic); the switch reaches it
 * through an edge interrupt that only counts, received messages through a static queue. After
 * start(), nothing allocates except std::string fields longer than 15 characters in the
 * latch's messages (its cause), which start() reserves room for.
 *
 *     static estop::Node estop_node({.name = "hmi", .input = {.gpio_num = GPIO_NUM_2}});
 *     estop_node.start();                  // app_main
 *     estop_node.attach(participant);      // once RTPS is up
 *     if (estop_node.active()) { stop(); } // anywhere, any task
 */

#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <optional>
#include <span>
#include <string_view>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "estop/core/codec.hpp"
#include "estop/core/latch_logic.hpp"
#include "estop/core/node_logic.hpp"
#include "estop/detail/edge_input.hpp"
#include "messages/estop_message.hpp"

namespace espp {
class RtpsParticipant;
}

namespace estop {

namespace detail {
struct Rx {
  core::Time rx;
  core::Decoded msg;
};
inline constexpr uint32_t kStackBytes = 4096;
inline constexpr UBaseType_t kRxQueueDepth = 8;
inline constexpr UBaseType_t kCommandQueueDepth = 4;
struct Command {
  enum class Kind : uint8_t { ENGAGE, CLEAR_ATTEMPT, SET_INPUT };
  Kind kind;
  Input input;
  char reason[16];
};
} // namespace detail

/// A Node's state, as a copy any task can hold.
struct NodeStatus {
  bool active = true;         ///< do not drive: the same as Node::active()
  bool engaged = false;       ///< this node asks for the stop, until the latch's clear attempt
  bool latch_engaged = false; ///< the latch says the stop is latched (and is fresh)
  bool latch_fresh = false;   ///< an e-stop message from the latch within the timeout
  bool armed = false;         ///< the switch interrupt is set up (true when there is no switch)
  Input input;                ///< the switch input in use
  uint32_t presses = 0;       ///< switch edges and engage() calls
  uint32_t clear_attempts_accepted = 0; ///< latch clear attempts this node accepted
  uint32_t clear_attempts_refused = 0;  ///< stale, switch held, or older than the press
  uint32_t rx_errors = 0;               ///< received messages that did not decode or fit
  uint32_t tx_errors = 0;               ///< sends refused after attach()
  uint32_t pub_seq = 0;                 ///< the last pub_seq sent
  char latch_source[16] = {};           ///< the latch's message, while it says ENGAGED
  char latch_reason[48] = {};
};

/// A node that can request the stop: its switch (optional) engages it, it sends its
/// state every period, and only the latch's clear attempt ends its request.
class Node {
public:
  struct Config {
    const char *name;           ///< EStop.source: 15 characters or fewer, unique per node
    Input input{};              ///< the e-stop switch; gpio_num GPIO_NUM_NC = none (engage() only)
    bool require_latch = false; ///< no message from the latch within the timeout = active()
    rammp::EStopPolicy policy = rammp::kEStopPolicy;
    /// This task decides and queues each send. The send itself runs on espp's RTPS
    /// tasks, at CONFIG_RTPS_THREAD_POOL_READER_PRIO (README, step 4), not at this one.
    UBaseType_t task_priority = 21;
    BaseType_t task_core = 0;
  };

  explicit Node(const Config &config);
  Node(const Node &) = delete;
  Node &operator=(const Node &) = delete;

  /// Arms the switch and starts the task. Once, from app_main. False = it could not
  /// start (a name longer than 15 characters, or no task), and active() stays true.
  bool start();

  /// Adds the e-stop writer and reader to `participant`, which must outlive the node.
  /// Once, after start(). Until then the node runs, and its messages go nowhere.
  bool attach(espp::RtpsParticipant &participant);

  /// Requests the stop as a press does, with `reason` (15 characters or fewer) in its
  /// messages. Any task; not an ISR.
  void engage(const char *reason);

  /// Moves the switch to `input` (gpio_num GPIO_NUM_NC = none), e.g. from a setting. A
  /// switch already held there is a press. Any task; waits if four commands are queued.
  void set_input(const Input &input);

  /// True while driving must not happen: this node engaged it, the latch says the stop
  /// is latched, the switch is not armed, the task is not running, or (require_latch)
  /// the latch is silent. Any task, an ISR included; one atomic load.
  bool active() const { return active_.load(std::memory_order_relaxed); }

  /// A consistent copy of the state. Any task.
  NodeStatus status() const;

private:
  static void task_main(void *self);
  void run();
  void on_sample(std::span<const uint8_t> payload);
  void send(core::Time now);
  void handle(const detail::Command &command, core::Time now);

  Config config_;
  core::NodeLogic logic_;
  detail::EdgeInput input_;
  char reason_[16] = {};
  std::atomic<bool> engage_pending_{false}; ///< engage() found the command queue full
  std::atomic<bool> active_{true};
  std::atomic<uint32_t> rx_dropped_{0};
  std::atomic<espp::RtpsParticipant *> participant_{nullptr};
  NodeStatus status_;
  rammp::EStop tx_;
  std::array<uint8_t, core::kMaxEncoded> tx_buf_{};

  TaskHandle_t task_ = nullptr;
  StaticTask_t task_tcb_;
  StackType_t task_stack_[detail::kStackBytes];
  QueueHandle_t rx_queue_ = nullptr;
  StaticQueue_t rx_queue_buf_;
  uint8_t rx_queue_storage_[detail::kRxQueueDepth * sizeof(detail::Rx)];
  QueueHandle_t command_queue_ = nullptr;
  StaticQueue_t command_queue_buf_;
  uint8_t command_queue_storage_[detail::kCommandQueueDepth * sizeof(detail::Command)];
  QueueHandle_t status_queue_ = nullptr; ///< a mailbox: overwritten by the task, peeked
  StaticQueue_t status_queue_buf_;
  uint8_t status_queue_storage_[sizeof(NodeStatus)];
};

/// The nodes a latch watches unless told otherwise: the HMI.
inline constexpr std::string_view kDefaultWatched[] = {"hmi"};

/// A Latch's state, as a copy any task can hold.
struct LatchStatus {
  bool latched = true; ///< the stop is in force: the same as Latch::active()
  bool clear_attempt_running = false;
  bool armed = false;          ///< the switch interrupt is set up (true when none)
  bool clear_armed = false;    ///< the clear input is set up (true when none)
  char cause[48] = "boot";     ///< why it latched
  uint32_t clear_attempts = 0; ///< attempts started
  uint32_t releases = 0;       ///< stops ended
  uint32_t rx_errors = 0;      ///< received messages that did not decode or fit
  uint32_t tx_errors = 0;      ///< sends refused after attach()
  uint32_t pub_seq = 0;        ///< the last pub_seq sent
};

/// The node that owns the latch (rammp::kEStopClearer). Latched from start(); latches on
/// any ENGAGED or a silent watched node; a clear attempt (the clear input held, or
/// start_clear_attempt()) releases it once every node has answered.
class Latch {
public:
  struct Config {
    /// Nodes the deadman expects to hear. The names must outlive the latch.
    std::span<const std::string_view> watched = kDefaultWatched;
    Input input{};       ///< a local e-stop switch; gpio_num GPIO_NUM_NC = none
    Input clear_input{}; ///< the clear button (held for clear_hold); GPIO_NUM_NC = none
    std::chrono::milliseconds clear_hold{2000};
    rammp::EStopPolicy policy = rammp::kEStopPolicy;
    UBaseType_t task_priority = 21;
    BaseType_t task_core = 0;
    bool log_events = true; ///< ESP_LOGI each latch change (tag "estop")
  };

  explicit Latch(const Config &config);
  Latch(const Latch &) = delete;
  Latch &operator=(const Latch &) = delete;

  /// Arms the inputs and starts the task. Once, from app_main.
  bool start();

  /// Adds the e-stop writer and reader to `participant`, which must outlive the latch.
  bool attach(espp::RtpsParticipant &participant);

  /// The operator asks to end the stop (the same as holding the clear input). Any task.
  void start_clear_attempt();

  /// This node's own reason to stop (15 characters or fewer). Any task; not an ISR.
  void engage(const char *reason);

  /// True while the stop is latched, or the task is not running. Any task; one load.
  bool active() const { return active_.load(std::memory_order_relaxed); }

  /// A consistent copy of the state. Any task.
  LatchStatus status() const;

private:
  static void task_main(void *self);
  void run();
  void on_sample(std::span<const uint8_t> payload);
  void send(core::Time now);
  void handle(const detail::Command &command, core::Time now);
  void poll_clear_input(core::Time now, bool switch_held);
  bool switch_held() const;

  Config config_;
  core::LatchLogic logic_;
  detail::EdgeInput input_;
  detail::EdgeInput clear_input_;
  std::optional<core::Time> clear_since_; ///< when the clear input went down
  bool clear_used_ = false;               ///< this hold already started an attempt
  char reason_[16] = {};
  std::atomic<bool> engage_pending_{false}; ///< engage() found the command queue full
  std::atomic<bool> active_{true};
  std::atomic<uint32_t> rx_dropped_{0};
  std::atomic<espp::RtpsParticipant *> participant_{nullptr};
  LatchStatus status_;
  rammp::EStop tx_;
  std::array<uint8_t, core::kMaxEncoded> tx_buf_{};

  TaskHandle_t task_ = nullptr;
  StaticTask_t task_tcb_;
  StackType_t task_stack_[detail::kStackBytes];
  QueueHandle_t rx_queue_ = nullptr;
  StaticQueue_t rx_queue_buf_;
  uint8_t rx_queue_storage_[detail::kRxQueueDepth * sizeof(detail::Rx)];
  QueueHandle_t command_queue_ = nullptr;
  StaticQueue_t command_queue_buf_;
  uint8_t command_queue_storage_[detail::kCommandQueueDepth * sizeof(detail::Command)];
  QueueHandle_t status_queue_ = nullptr;
  StaticQueue_t status_queue_buf_;
  uint8_t status_queue_storage_[sizeof(LatchStatus)];
};

} // namespace estop
