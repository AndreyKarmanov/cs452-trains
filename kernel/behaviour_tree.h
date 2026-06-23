#pragma once

#include "buffer.h"
#include "clock_server.h"
#include "io_helpers.h"
#include "mrk.h"
#include "name_server.h"
#include "train_control.h"
#include "train_state.h"
#include "uart_tx_server.h"

struct Blackboard {
  State state;
  uint32_t loco_id;

  int tx_server_tid;
  int cs_server_tid;

  uint16_t estimated_speed;
  uint16_t requested_speed;

  uint16_t last_seen_sensor;
  uint16_t expected_next_sensor;

  MRKCmd new_event;

  StaticString<32> error_msg;
};

enum class NodeResult {
  Success,
  Failure,
  Running,
};

// https://wboayue.com/posts/behavior-trees/
// https://www.behaviortree.dev/docs/learn-the-basics/BT_basics
struct TreeNode {
  virtual ~TreeNode()                     = default;
  virtual NodeResult tick(Blackboard &bb) = 0;
};

struct DecoratorNode : public TreeNode {
  TreeNode *child;
};

struct ControlNode : public TreeNode {
  Buffer<TreeNode *, 8> children;
};

struct FallBackNode : public ControlNode {
  NodeResult tick(Blackboard &bb) override {
    for (TreeNode *child : children) {
      NodeResult result = child->tick(bb);
      if (result != NodeResult::Failure) {
        return result;
      }
    }
    return NodeResult::Failure;
  }
};

struct SequenceNode : public ControlNode {
  NodeResult tick(Blackboard &bb) override {
    for (TreeNode *child : children) {
      NodeResult result = child->tick(bb);
      if (result != NodeResult::Success) {
        return result;
      }
    }
    return NodeResult::Success;
  }
};

struct LeafNode : public TreeNode {};

struct WaitNode : public LeafNode {
  uint32_t wait_ticks;
  uint32_t start_tick;

  WaitNode(uint32_t wait_ticks) : wait_ticks(wait_ticks), start_tick(0) {}

  NodeResult tick(Blackboard &bb) override {
    if (start_tick == 0) {
      start_tick = Time(bb.cs_server_tid);
    }
    if (Time(bb.cs_server_tid) - start_tick >= wait_ticks) {
      return NodeResult::Success;
    }
    return NodeResult::Running;
  }
};

struct ActionNode : public LeafNode {
  NodeResult (*action)(Blackboard &);

  ActionNode(NodeResult (*action)(Blackboard &)) : action(action) {}

  NodeResult tick(Blackboard &bb) override { return action(bb); }
};

inline void test_tree() {
  Blackboard bb{.loco_id = 17};

  auto cs_tid = WhoIs(ClockServer<>::CLOCK_SERVER_NAME);
  _assert(cs_tid >= 0, "CLOCK SERVER WHOIS FAILED");

  auto tx_tid = WhoIs(UART_TX_Server::TX_SERVER_NAME);
  _assert(tx_tid >= 0, "TX SERVER WHOIS FAILED");

  auto tcs_tid = WhoIs(TrainControlServer<>::TC_SERVER_NAME);
  _assert(tcs_tid >= 0, "TC SERVER WHOIS FAILED");

  bb.cs_server_tid = cs_tid;
  bb.tx_server_tid = tx_tid;

  auto tree = SequenceNode{};

  WaitNode wait_node(5000);
  tree.children.push(&wait_node);

  ActionNode action_node([](Blackboard &bb) {
    Puts(bb.tx_server_tid, "Action node executed\n\r");
    return NodeResult::Success;
  });
  tree.children.push(&action_node);

  while (true) {
    NodeResult result = tree.tick(bb);
    if (result == NodeResult::Success) {
      Puts(bb.tx_server_tid, "Tree completed successfully\n\r");
      break;
    } else if (result == NodeResult::Failure) {
      Puts(bb.tx_server_tid, "Tree failed\n\r");
      break;
    }
  }
}