#pragma once

#include "buffer.h"
#include "mrk.h"
#include "pathfind.h"
#include "train_state.h"
#include <cstdint>

struct Blackboard {
  int tcs_tid{0};
  int txs_tid{0};
  int cs_tid{0};

  Pathfind pathfinder;
  State state{};
  uint32_t loco_id{};
  TrainState *loco{nullptr};

  Path path{};

  struct SensorSighting {
    uint16_t sid;
    uint32_t tick;
  };
  Buffer<SensorSighting, TRACK_MAX> seen_sensors{};

  struct DistLog {
    uint16_t dx_prev;
    uint32_t tick;
    SensorData sensor_data;
  };
  Buffer<DistLog, TRACK_MAX> dists{};

  uint16_t target_speed{0};

  int dx_next_sens_um{0};

  MRKCmd new_event{};
  uint32_t curr_tick{0};
  uint32_t last_tick{0};
  uint32_t lookahead_um{0};

  uint32_t saved_tick{0};
  uint32_t top_loop_time{0};
  uint32_t accel_loop_time{0};

  StaticString<8> nav_goal{};
  StaticString<32> error_msg{};
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

struct InvertNode : public DecoratorNode {
  InvertNode(TreeNode *child) : DecoratorNode(child) {}
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

struct RepeatNode : public DecoratorNode {
  int times              = 0;
  NodeResult last_result = NodeResult::Success;
  RepeatNode(TreeNode *child, int times) : DecoratorNode(child), times(times) {}
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