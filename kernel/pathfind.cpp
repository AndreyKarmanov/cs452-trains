#include "pathfind.h"
#include "debug.h"
#include "heap.h"
#include "track_data.h"
#include "uart.h"
#include <climits>
#include <cstdint>

static constexpr int INF = INT_MAX / 2;

Path &Path::operator+(const Path &other) {
  if (other.empty())
    return *this;

  if (this->empty()) {
    *this = other;
    return *this;
  }
  auto last_opt  = this->peek_last();
  auto first_opt = other.peek();

  if (!last_opt.has_value() || !first_opt.has_value()) {
    _assert(false, "empty path");
    return *this;
  }

  if (last_opt->node_idx != first_opt->node_idx) {
    _assert(false, "other must start at last node of this");
    return *this;
  }

  if (this->size() + other.size() - 1 > TRACK_MAX) {
    _assert(false, "Path overflow");
    return *this;
  }

  this->dist_mm                  += other.dist_mm;
  (*(this->end() - 1)).dx_next    = first_opt->dx_next;
  (*(this->end() - 1)).br_curved  = first_opt->br_curved;

  for (size_t i = 1; i < other.size(); ++i) {
    auto node = other[i];
    if (!node.has_value()) {
      _assert(false, "unexpected empty path node");
      return *this;
    }
    this->push(node.value());
  }

  return *this;
}

Track::Track(Track::Layout layout) {

  if (layout == Track::Layout::A) {
    init_tracka(track);
  } else {
    init_trackb(track);
  }

  for (int node_idx = 0; node_idx < TRACK_MAX; ++node_idx) {
    if (track[node_idx].name != nullptr && track[node_idx].name[0] != '\0')
      node_to_idx.set(track[node_idx].name, node_idx);
  }
}

void Track::reserve(int node_idx, int dir, uint32_t id) {
  auto &edge                = track[node_idx].edge[dir];
  edge.res_loco_id          = id;
  edge.reverse->res_loco_id = id;
}

void Track::release(int node_idx, int dir, uint32_t id) {
  auto &edge = track[node_idx].edge[dir];
  if (edge.res_loco_id == UNRESERVED ||
      static_cast<uint32_t>(edge.res_loco_id) != id) {
    return;
  }
  edge.res_loco_id          = UNRESERVED;
  edge.reverse->res_loco_id = UNRESERVED;
}

bool Track::has_reservation(const PathNode &node, uint32_t loco_id) {
  return track[node.node_idx].edge[node.br_curved].res_loco_id == loco_id;
}

uint32_t Track::get_reservation(int node_idx, int dir) {
  return track[node_idx].edge[dir].res_loco_id;
}

std::optional<int> Track::get_idx(const StaticString<4> &name) const {
  return node_to_idx.get(name);
}

std::optional<track_edge> Track::get_edge(int from_idx, int to_idx) const {
  const track_node &from    = track[from_idx];
  const track_node &to_node = track[to_idx];

  if (from.reverse == &to_node)
    return std::nullopt;

  switch (from.type) {
  case NODE_SENSOR:
  case NODE_MERGE:
  case NODE_ENTER:
    if (from.edge[DIR_AHEAD].dest == &to_node)
      return from.edge[DIR_AHEAD];
    break;

  case NODE_BRANCH:
    if (from.edge[DIR_STRAIGHT].dest == &to_node)
      return from.edge[DIR_STRAIGHT];
    if (from.edge[DIR_CURVED].dest == &to_node)
      return from.edge[DIR_CURVED];
    break;

  default:
    break;
  }

  return std::nullopt;
}

std::optional<Path> Track::build_path(int goal_idx,
                                      const int best_dist[TRACK_MAX],
                                      const int predecessor[TRACK_MAX]) const {

  if (best_dist[goal_idx] >= INF)
    return std::nullopt;

  size_t path_len = 0;
  for (int node_idx = goal_idx; node_idx != -1;
       node_idx     = predecessor[node_idx])
    ++path_len;

  std::array<int, TRACK_MAX> node_indices{};
  size_t write_idx = path_len;
  for (int node_idx = goal_idx; node_idx != -1;
       node_idx     = predecessor[node_idx])
    node_indices[--write_idx] = node_idx;

  Path result{};
  result.dist_mm = best_dist[goal_idx];

  for (size_t step = 0; step < path_len; ++step) {
    int node_idx           = node_indices[step];
    const track_node &node = track[node_idx];

    int dist_to_prev = 0;
    int dist_to_next = 0;
    int edge_v_pct   = 100;
    bool curved      = false;

    if (step > 0) {
      int prev_idx = node_indices[step - 1];
      auto edge    = get_edge(prev_idx, node_idx).value_or({});
      dist_to_prev = edge.dist;
      edge_v_pct   = edge.edge_v_pct;
    }
    if (step + 1 < path_len) {
      int next_idx = node_indices[step + 1];
      auto edge    = get_edge(node_idx, next_idx).value_or({});
      dist_to_next = edge.dist;
      if (node.type == NODE_BRANCH) {
        curved = (node.edge[DIR_CURVED].dest == &track[next_idx]);
      }
    }

    result.push({.node_idx   = node_idx,
                 .type       = node.type,
                 .num        = node.num,
                 .dx_prev    = dist_to_prev,
                 .dx_next    = dist_to_next,
                 .edge_v_pct = edge_v_pct,
                 .br_curved  = curved});
  }

  return result;
}

std::optional<Path> Track::find_loop(int start_idx) const {
  // if the start and goal is the same
  // for each potential path from this node, find the shortest path that loops
  // back to goal node, then append on
  const auto &node = track[start_idx];
  auto goal_idx    = start_idx;
  if (node.type == NODE_BRANCH) {
    auto forward_idx = node_idx(node.edge[DIR_STRAIGHT].dest);
    auto curved_idx  = node_idx(node.edge[DIR_CURVED].dest);

    auto straight_p_o1 = find_path(start_idx, forward_idx, false);
    auto straight_p_o2 = find_path(forward_idx, goal_idx, false);

    auto curved_p_o1 = find_path(start_idx, curved_idx, false);
    auto curved_p_o2 = find_path(curved_idx, goal_idx, false);

    if (straight_p_o1.has_value() && straight_p_o2.has_value()) {
      return straight_p_o1.value() + straight_p_o2.value();
    } else if (curved_p_o1.has_value() && curved_p_o2.has_value()) {
      return curved_p_o1.value() + curved_p_o2.value();
    } else {
      return std::nullopt;
    }
  } else if (node.type == NODE_SENSOR || node.type == NODE_MERGE ||
             node.type == NODE_ENTER) {
    auto ahead_idx  = node_idx(node.edge[DIR_AHEAD].dest);
    auto ahead_p_o1 = find_path(start_idx, ahead_idx, false);
    auto ahead_p_o2 = find_path(ahead_idx, goal_idx, false);
    if (!ahead_p_o1.has_value() || !ahead_p_o2.has_value())
      return std::nullopt;
    return ahead_p_o1.value() + ahead_p_o2.value();
  } else {
    return std::nullopt;
  }
}

std::optional<Path> Track::find_path(int start_idx, int goal_idx,
                                     bool allow_reverse) const {
  if (start_idx < 0 || start_idx >= TRACK_MAX || goal_idx < 0 ||
      goal_idx >= TRACK_MAX)
    return std::nullopt;

  if (start_idx == goal_idx) {
    return find_loop(start_idx);
  }

  int best_dist[TRACK_MAX];
  int predecessor[TRACK_MAX];

  for (int node_idx = 0; node_idx < TRACK_MAX; ++node_idx) {
    best_dist[node_idx]   = INF;
    predecessor[node_idx] = -1;
  }

  best_dist[start_idx] = 0;
  Heap<std::pair<int, int>, TRACK_MAX> frontier;
  frontier.push({0, start_idx});

  auto relax = [&](int from_idx, int from_dist, int to_idx, int edge_dist) {
    int new_dist = from_dist + edge_dist;
    if (new_dist < best_dist[to_idx]) {
      best_dist[to_idx]   = new_dist;
      predecessor[to_idx] = from_idx;
      frontier.push({new_dist, to_idx});
    }
  };

  while (!frontier.empty()) {
    auto [pop_dist, curr_idx] = frontier.pop().value();
    if (pop_dist > best_dist[curr_idx])
      continue;
    if (curr_idx == goal_idx)
      break;

    const track_node &curr_node = track[curr_idx];
    int curr_dist               = best_dist[curr_idx];

    switch (curr_node.type) {
    case NODE_SENSOR:
    case NODE_MERGE:
    case NODE_ENTER:
      relax(curr_idx, curr_dist, node_idx(curr_node.edge[DIR_AHEAD].dest),
            curr_node.edge[DIR_AHEAD].dist);
      break;

    case NODE_BRANCH:
      relax(curr_idx, curr_dist, node_idx(curr_node.edge[DIR_STRAIGHT].dest),
            curr_node.edge[DIR_STRAIGHT].dist);
      relax(curr_idx, curr_dist, node_idx(curr_node.edge[DIR_CURVED].dest),
            curr_node.edge[DIR_CURVED].dist);
      break;

    case NODE_EXIT:
      break;

    default:
      break;
    }

    if (allow_reverse) {
      relax(curr_idx, curr_dist, node_idx(curr_node.reverse), REVERSE_COST);
    }
  }

  return build_path(goal_idx, best_dist, predecessor);
}

std::optional<Path> Track::find_path(const StaticString<4> &from,
                                     const StaticString<4> &to,
                                     bool allow_reverse) const {
  auto start_idx = get_idx(from);
  auto goal_idx  = get_idx(to);
  if (!start_idx.has_value() || !goal_idx.has_value())
    return std::nullopt;

  return find_path(start_idx.value(), goal_idx.value(), allow_reverse);
}

// returns number of nodes found
int Track::search_within_distance(int start_idx, int distance, int *result,
                                  int length, bool allow_reverse) {
  int best_dist[TRACK_MAX];
  int predecessor[TRACK_MAX];

  for (int i = 0; i < TRACK_MAX; ++i) {
    best_dist[i]   = INF;
    predecessor[i] = -1;
  }

  best_dist[start_idx] = 0;
  Heap<std::pair<int, int>, TRACK_MAX> frontier;
  frontier.push({0, start_idx});

  auto relax = [&](int from_idx, int from_dist, int to_idx, int edge_dist) {
    int new_dist = from_dist + edge_dist;
    if (new_dist < best_dist[to_idx]) {
      best_dist[to_idx]   = new_dist;
      predecessor[to_idx] = from_idx;
      frontier.push({new_dist, to_idx});
    }
  };

  int result_count = 0;
  while (!frontier.empty() && result_count < length) {
    auto [pop_dist, curr_idx] = frontier.pop().value();
    if (pop_dist > best_dist[curr_idx])
      continue;
    if (pop_dist > distance)
      break;

    result[result_count++] = curr_idx;

    const track_node &curr_node = track[curr_idx];
    int curr_dist               = best_dist[curr_idx];

    switch (curr_node.type) {
    case NODE_SENSOR:
    case NODE_MERGE:
    case NODE_ENTER:
      relax(curr_idx, curr_dist, node_idx(curr_node.edge[DIR_AHEAD].dest),
            curr_node.edge[DIR_AHEAD].dist);
      break;

    case NODE_BRANCH:
      relax(curr_idx, curr_dist, node_idx(curr_node.edge[DIR_STRAIGHT].dest),
            curr_node.edge[DIR_STRAIGHT].dist);
      relax(curr_idx, curr_dist, node_idx(curr_node.edge[DIR_CURVED].dest),
            curr_node.edge[DIR_CURVED].dist);
      break;

    case NODE_EXIT:
      break;

    default:
      break;
    }

    if (allow_reverse) {
      relax(curr_idx, curr_dist, node_idx(curr_node.reverse), REVERSE_COST);
    }
  }

  return result_count;
}

const char *Track::node_name(int node_idx) const {
  if (node_idx < 0 || node_idx >= TRACK_MAX)
    return "?";
  const char *name = track[node_idx].name;
  return (name != nullptr && name[0] != '\0') ? name : "?";
}

static const char *node_type_name(node_type type) {
  switch (type) {
  case NODE_SENSOR:
    return "SENSOR";
  case NODE_BRANCH:
    return "BRANCH";
  case NODE_MERGE:
    return "MERGE";
  case NODE_ENTER:
    return "ENTER";
  case NODE_EXIT:
    return "EXIT";
  default:
    return "NONE";
  }
}

static void print_path(const Track &pathfind, const char *label,
                       const std::optional<Path> &path_opt) {
  if (!path_opt.has_value()) {
    debug_printf(CONSOLE, "%s: no path\n\r", label);
    return;
  }

  auto path = path_opt.value();

  debug_printf(CONSOLE, "%s: dist=%d len=%d ", label, path.dist_mm,
               static_cast<int>(path.size()));
  for (size_t step = 0; step < path.size(); ++step) {
    auto node = path[step];
    if (!node.has_value())
      continue;
    debug_printf(CONSOLE, "%s", pathfind.node_name(node->node_idx));
    if (step + 1 < path.size())
      debug_puts(CONSOLE, " -> ");
  }
  debug_puts(CONSOLE, "\n\r");

  for (size_t step = 0; step < path.size(); ++step) {
    auto node = path[step];
    if (!node.has_value()) {
      continue;
    }
    debug_printf(CONSOLE,
                 "  [%d] node_idx=%d name=%s type=%s dist_prev=%d dist_next=%d "
                 "curved=%d\n\r",
                 static_cast<int>(step), node->node_idx,
                 pathfind.node_name(node->node_idx), node_type_name(node->type),
                 node->dx_prev, node->dx_next, node->br_curved ? 1 : 0);
  }
}

void test_pathfind() {
  debug_puts(CONSOLE, "pathfind tests\n\r");

  Track track_a(Track::Layout::A);
  print_path(track_a, "A1->A13", track_a.find_path("A1", "A13", true));
  print_path(track_a, "A13->A1", track_a.find_path("A13", "A1"));
  print_path(track_a, "A1->E16", track_a.find_path("A1", "E16"));
  print_path(track_a, "A1->A1", track_a.find_path("A1", "A1"));
  print_path(track_a, "A1->ZZZ", track_a.find_path("A1", "ZZZ"));
  print_path(track_a, "A13->B6", track_a.find_path("A13", "B6"));

  Track track_b(Track::Layout::B);
  print_path(track_b, "C10->B16", track_b.find_path("C10", "B16"));
  print_path(track_b, "C10->C10", track_b.find_path("C10", "C10"));
  print_path(track_b, "C13->A11", track_b.find_path(44, 10));

  debug_puts(CONSOLE, "pathfind tests done\n\r");
}
