#pragma once

#include "buffer.h"
#include "clock_server.h"
#include "mrk.h"
#include "train_state.h"

struct Blackboard {
  State state;
  uint32_t loco_id;

  int tcs_tid;
  int txs_tid;
  int cs_tid;

  uint16_t est_speed;
  uint16_t requested_speed;

  uint16_t last_seen_sensor;
  uint16_t expected_next_sensor;

  MRKCmd new_event;
  uint32_t event_tick;

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
      start_tick = Time(bb.cs_tid);
    }
    if (Time(bb.cs_tid) - start_tick >= wait_ticks) {
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