#pragma once

#include "buffer.h"
#include "heap.h"
#include "map.h"
#include "static_string.h"
#include "track_data.h"
#include <array>
#include <cstddef>
#include <optional>

struct PathNode {
  int node_idx;
  node_type type;
  int distance_to_next_node;
  bool should_br_be_curved;

  bool operator==(const PathNode &other) const {
    return node_idx == other.node_idx && type == other.type &&
           should_br_be_curved == other.should_br_be_curved;
  }
};

class Path : public Buffer<PathNode, TRACK_MAX> {
public:
  int dist = 0;

  // in place addition of two paths
  Path &operator+(const Path &other);

  // search all nodes within distance.
};

class Pathfind {
  Map<StaticString<8>, int, TRACK_MAX> node_to_idx;

  int node_index(const track_node *node) const {
    return static_cast<int>(node - track);
  }

  bool can_visit(int node_idx) const;

  int edge_dist_between(int from_idx, int to_idx) const;

  void relax(int from_idx, int from_dist, int to_idx, int edge_dist,
             int best_dist[TRACK_MAX], int predecessor[TRACK_MAX],
             Heap<std::pair<int, int>, TRACK_MAX> &frontier) const;

  std::optional<Path> build_path(int start_idx, int goal_idx,
                                 const int best_dist[TRACK_MAX],
                                 const int predecessor[TRACK_MAX]) const;

public:
  static track_node track[TRACK_MAX];
  static constexpr int REVERSE_COST = 500;

  explicit Pathfind(char track_layout);

  std::optional<int> get_idx(const char *name) const;

  std::optional<Path> shortest_path(int start_idx, int goal_idx,
                                    bool allow_reverse = false) const;

  std::optional<Path> shortest_path(const char *from, const char *to,
                                    bool allow_reverse = false) const;

  bool is_curved(int from_idx, int to_idx) const;

  const char *node_name(int node_idx) const;
};

void test_pathfind();
