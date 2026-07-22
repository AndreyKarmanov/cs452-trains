#pragma once

#include "buffer.h"
#include "mrk.h"
#include "pathfind.h"
#include "train_state.h"
#include <cstdint>

struct Blackboard {

  // tree tick data
  // tree state
  MRKCmd new_event{UnknownCmd{}};
  uint32_t curr_tick{0};
  StaticString<32> error_msg{};

  // tids for servers
  int tcs_tid{0};
  int txs_tid{0};
  int web_tid{0};

  // track & train data
  Track track{Track::Layout::A};
  State state{};
  TrainState *loco{nullptr};

  Path path{};

  // distance logs for calibraiton
  struct DistLog {
    SensorData from;
    SensorData to;
    int d_um;
    uint32_t d_t;
  };
  Buffer<DistLog, TRACK_MAX> dists{};
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

struct LeafNode : public TreeNode {};

struct DecoratorNode : public TreeNode {
  TreeNode *child;
  DecoratorNode(TreeNode *child) : child(child) {}
};

struct ControlNode : public TreeNode {
  Buffer<TreeNode *, 16> children;
};

struct Fallback : public ControlNode {
  Fallback() = default;
  Fallback(std::initializer_list<TreeNode *> init) {
    for (TreeNode *n : init) {
      children.push(n);
    }
  }

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

struct Sequence : public ControlNode {

  Sequence() = default;
  Sequence(std::initializer_list<TreeNode *> init) {
    for (TreeNode *n : init) {
      children.push(n);
    }
  }

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

struct Invert : public DecoratorNode {
  Invert(TreeNode *child) : DecoratorNode(child) {}
  NodeResult tick(Blackboard &bb) override {
    NodeResult result = child->tick(bb);
    if (result == NodeResult::Success) {
      return NodeResult::Failure;
    } else if (result == NodeResult::Failure) {
      return NodeResult::Success;
    }
    return result;
  }
};

struct RepeatForeverNode : public DecoratorNode {
  NodeResult tick(Blackboard &bb) override {
    while (true) {
      NodeResult result = child->tick(bb);
      if (result == NodeResult::Failure) {
        return NodeResult::Failure;
      } else {
        return NodeResult::Running;
      }
    }
    return NodeResult::Success;
  }
};

struct Repeat : public DecoratorNode {
  int times              = 0;
  NodeResult last_result = NodeResult::Success;
  Repeat(TreeNode *child, int times) : DecoratorNode(child), times(times) {}
  NodeResult tick(Blackboard &bb) override {
    if (times == 0) {
      return last_result;
    }
    last_result = child->tick(bb);

    if (last_result == NodeResult::Success ||
        last_result == NodeResult::Failure) {
      times--;
    }
    if (times > 0) {
      return NodeResult::Running;
    }
    return last_result;
  }
};

struct TimeoutNode : public DecoratorNode {
  uint32_t start_tick = 0;
  uint32_t timeout    = 0;
  TimeoutNode(TreeNode *child, int timeout)
      : DecoratorNode(child), timeout(timeout) {}
  NodeResult tick(Blackboard &bb) override {
    NodeResult result = child->tick(bb);

    if (result != NodeResult::Running) {
      start_tick = 0;
      return result;
    }

    if (start_tick == 0) {
      start_tick = bb.curr_tick;
    } else if (bb.curr_tick - start_tick > timeout) {
      return NodeResult::Failure;
    }
    return NodeResult::Running;
  }
};

struct WaitNode : public LeafNode {
  uint32_t wait_ticks{0};
  uint32_t start_tick{0};

  WaitNode(uint32_t wait_ticks) : wait_ticks(wait_ticks), start_tick(0) {}

  NodeResult tick(Blackboard &bb) override {
    if (start_tick == 0) {
      start_tick = bb.curr_tick;
      return NodeResult::Running;
    }
    if (bb.curr_tick - start_tick >= wait_ticks) {
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