#pragma once

#include "buffer.h"
#include "map.h"
#include "static_string.h"
#include "track_data.h"
#include <optional>

// todo: change this so that it's pointers to nodes, not sure we need type /
// node
struct PathNode {
  int node_idx;
  node_type type;
  int num;
  int dx_prev{0};
  int dx_next{0};
  int edge_v_pct{100}; // how much higher / lower the velocity can get on this
                       // edge relative to max
  bool br_curved{false};
  bool reserved{false};

  bool operator==(const PathNode &other) const {
    return node_idx == other.node_idx && type == other.type &&
           dx_prev == other.dx_prev && dx_next == other.dx_next &&
           br_curved == other.br_curved;
  }
};

class Path : public Buffer<PathNode, TRACK_MAX> {
public:
  int dist_mm = 0;

  // in place addition of two paths
  Path &operator+(const Path &other);

  constexpr void pop(int n) {
    for (int i = 0; i < n; i++) {
      auto elem = Buffer<PathNode, TRACK_MAX>::pop();
      if (!elem.has_value())
        break;
      dist_mm -= elem->dx_next;
    }
  }

  constexpr std::optional<PathNode> pop() {
    auto elem = Buffer<PathNode, TRACK_MAX>::pop();
    if (elem.has_value())
      dist_mm -= elem->dx_next;
    return elem;
  }

  int lookahead(int distance, PathNode *result, int length,
                int start_offset = 0, node_type node_type = NODE_NONE);
};

class Track {
  Map<StaticString<4>, int, TRACK_MAX> node_to_idx;

  std::optional<track_edge> get_edge(int from_idx, int to_idx) const;

  std::optional<Path> build_path(int goal_idx, const int best_dist[TRACK_MAX],
                                 const int predecessor[TRACK_MAX]) const;

  static track_node track[TRACK_MAX];

public:
  enum class Layout { A, B };
  static constexpr int REVERSE_COST = 500;

  explicit Track(Layout layout);

  void reserve(int node_idx, int dir, uint32_t id);
  void release(int node_idx, int dir, uint32_t id);
  bool has_reservation(const PathNode &node, uint32_t loco_id);
  uint32_t get_reservation(int node_idx, int dir);

  std::optional<int> get_idx(const StaticString<4> &name) const;
  int node_idx(const track_node *node) const {
    return static_cast<int>(node - track);
  }

  std::optional<Path> find_loop(int start_idx) const;
  std::optional<Path> find_path(int start_idx, int goal_idx,
                                bool allow_reverse = false) const;
  std::optional<Path> find_path(const StaticString<4> &start,
                                const StaticString<4> &goal,
                                bool allow_reverse = false) const;

  // search all nodes within distance.
  int search_within_distance(int node_idx, int distance, int *result,
                             int length, bool allow_reverse = false);

  const char *node_name(int node_idx) const;

  const track_node operator[](int idx) const { return track[idx]; }
};

void test_pathfind();
