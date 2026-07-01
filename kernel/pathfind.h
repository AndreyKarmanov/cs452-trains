#pragma once

#include "buffer.h"
#include "map.h"
#include "static_string.h"
#include "track_data.h"
#include <optional>

struct PathNode {
  int node_idx;
  node_type type;
  int num;
  int dx_prev = 0;
  int dx_next;
  int edge_v_pct = 100; // how much higher / lower the velocity can get on this
                        // edge relative to max
  bool should_br_be_curved;

  bool operator==(const PathNode &other) const {
    return node_idx == other.node_idx && type == other.type &&
           dx_prev == other.dx_prev && dx_next == other.dx_next &&
           should_br_be_curved == other.should_br_be_curved;
  }
};

class Path : public Buffer<PathNode, TRACK_MAX> {
public:
  int dist = 0;

  // in place addition of two paths
  Path &operator+(const Path &other);

  constexpr void pop(int n) {
    for (int i = 0; i < n; i++) {
      auto elem = Buffer<PathNode, TRACK_MAX>::pop();
      if (!elem.has_value())
        break;
      dist -= elem->dx_next;
    }
  }

  constexpr std::optional<PathNode> pop() {
    auto elem = Buffer<PathNode, TRACK_MAX>::pop();
    if (elem.has_value())
      dist -= elem->dx_next;
    return elem;
  }

  int lookahead(int distance, PathNode *result, int length,
                int start_offset = 0, node_type node_type = NODE_NONE);
};

class Pathfind {
  Map<StaticString<8>, int, TRACK_MAX> node_to_idx;

  int node_index(const track_node *node) const {
    return static_cast<int>(node - track);
  }

  bool can_visit(int node_idx) const;

  int edge_dist_between(int from_idx, int to_idx) const;
  std::optional<track_edge> get_edge(int from_idx, int to_idx) const;

  std::optional<Path> build_path(int start_idx, int goal_idx,
                                 const int best_dist[TRACK_MAX],
                                 const int predecessor[TRACK_MAX]) const;

public:
  static track_node track[TRACK_MAX];
  static constexpr int REVERSE_COST = 500;

  explicit Pathfind(char track_layout);

  std::optional<int> get_idx(const char *name) const;

  std::optional<Path> shortest_loop(int start_idx) const;
  std::optional<Path> shortest_path(int start_idx, int goal_idx,
                                    bool allow_reverse = false) const;

  std::optional<Path> shortest_path(const char *from, const char *to,
                                    bool allow_reverse = false) const;

  // search all nodes within distance.
  int search_within_distance(int node_idx, int distance, int *result,
                             int length, bool allow_reverse = false);

  const char *node_name(int node_idx) const;
};

void test_pathfind();
