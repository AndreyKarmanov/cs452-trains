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
  int dx_prev{0};
  int dx_next{0};
  int edge_v_pct{100};
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
};

class Track {
public:
  typedef StaticString<8> NodeName;

private:
  Map<NodeName, int, TRACK_MAX> node_to_idx;


  std::optional<Path> build_path(int goal_idx, const int best_dist[TRACK_MAX],
                                 const int predecessor[TRACK_MAX]) const;

  track_node track[TRACK_MAX];

public:
  enum class Layout { A, B };
  static constexpr int REVERSE_COST = 500;

  explicit Track(Layout layout);

  void reserve(int node_idx, int dir, uint32_t id);
  void release(int node_idx, int dir, uint32_t id);
  bool has_reservation(const PathNode &node, uint32_t loco_id);
  uint32_t get_reservation(int node_idx, int dir);

  std::optional<int> get_idx(const NodeName &name) const;
  int node_idx(const track_node *node) const {
    return static_cast<int>(node - track);
  }
    std::optional<track_edge> get_edge(int from_idx, int to_idx) const;


  std::optional<Path> find_loop(int start_idx) const;
  std::optional<Path> find_path(int start_idx, int goal_idx,
                                bool allow_reverse = false) const;
  std::optional<Path> find_path(const NodeName &start, const NodeName &goal,
                                bool allow_reverse = false) const;

  const char *node_name(int node_idx) const;

  const track_node operator[](int idx) const { return track[idx]; }
  const track_node operator[](const NodeName &name) const {
    auto idx = get_idx(name);
    if (!idx.has_value())
      return track[0];
    return track[idx.value()];
  }
};

void test_pathfind();
