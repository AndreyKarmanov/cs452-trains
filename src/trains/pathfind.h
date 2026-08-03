#pragma once

#include "buffer.h"
#include "map.h"
#include "static_string.h"
#include "track_data.h"
#include <optional>

struct PathNode {
  int node_idx;
  node_type type;
  int dx_next{0};
  bool br_curved : 1 {false};
  bool has_reservation : 1 {false};

  bool operator==(const PathNode &other) const {
    return node_idx == other.node_idx && type == other.type &&
           br_curved == other.br_curved;
  }
};

struct PathLocation {
  int node_idx;
  bool br_curved;
  int pct;
};

class Track;

class Path : public Buffer<PathNode, TRACK_MAX> {
public:
  const Track *track;
  int dist_mm = 0;

  // in place addition of two paths
  Path &operator+(const Path &other);

  bool push(const PathNode &node);
  bool push_front(const PathNode &node);

  constexpr void pop(size_t n) {
    for (size_t i = 0; i < n; i++) {
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

  constexpr std::optional<PathNode> pop_back() {
    auto elem = Buffer<PathNode, TRACK_MAX>::pop_back();
    if (elem.has_value()) {
      dist_mm -= elem->dx_next;
      // set the last node's dx_next to 0 since it is now the last node
      if (!empty()) {
        dist_mm                -= (*(end() - 1)).dx_next;
        (*(end() - 1)).dx_next  = 0;
      }
    }
    return elem;
  }

  constexpr void clear() {
    Buffer<PathNode, TRACK_MAX>::clear();
    dist_mm = 0;
  }

  // Walk path forward; return first node where offset_um < segment length.
  std::optional<PathLocation> locate_at(int offset_um) const;

  StaticString<128> to_string(const Track *track) const;

  Path reverse();
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

  const track_node &operator[](int idx) const { return track[idx]; }
  const track_node &operator[](const NodeName &name) const {
    auto idx = get_idx(name);
    if (!idx.has_value())
      return track[0];
    return track[idx.value()];
  }
};

struct EncodedPath : public Buffer<uint8_t, TRACK_MAX> {
  EncodedPath() = default;
  EncodedPath(const Path &path) {
    for (const auto &node : path) {
      if (node.node_idx < 0 || node.node_idx >= TRACK_MAX) {
        _assert(false, "bad node index in path");
        break;
      }
      push(static_cast<uint8_t>(node.node_idx));
    }
  }
  Path decode(const Track &track) const {
    Path result{};
    result.track = &track;
    for (auto it = begin(); it != end(); ++it) {
      int node_idx = static_cast<int>(*it);
      if (node_idx < 0 || node_idx >= TRACK_MAX) {
        _assert(false, "bad node index in encoded path");
        continue;
      }

      const track_node &node = track[node_idx];
      int dx_next            = 0;
      bool curved            = false;

      if (it + 1 != end()) {
        int next_idx = static_cast<int>(*(it + 1));
        auto edge    = track.get_edge(node_idx, next_idx);
        if (!edge.has_value()) {
          _assert(false, "bad edge in encoded path");
          break;
        }
        dx_next = edge->dist;
        if (node.type == NODE_BRANCH) {
          curved = (node.edge[DIR_CURVED].dest == &track[next_idx]);
        }
      }
      result.push({.node_idx  = node_idx,
                   .type      = node.type,
                   .dx_next   = dx_next,
                   .br_curved = curved});
    }
    return result;
  }

  bool operator==(const EncodedPath &other) const = default;
};

void test_pathfind();
