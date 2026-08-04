#include "pathfind.h"
#include "debug.h"
#include "heap.h"
#include "track_data.h"
#include "uart.h"
#include <climits>

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

  this->pop_back();
  for (size_t i = 0; i < other.size(); ++i) {
    auto node = other[i];
    if (!node.has_value()) {
      _assert(false, "unexpected empty path node");
      return *this;
    }
    this->push(node.value());
  }

  return *this;
}

bool Path::push(const PathNode &node) {
  PathNode to_insert = node;

  if (empty()) {
    to_insert.dx_next = 0;
    return Buffer<PathNode, TRACK_MAX>::push(to_insert);
  }

  auto &tail       = *(end() - 1);
  int old_tail_dx  = tail.dx_next;
  bool old_tail_br = tail.br_curved;
  int new_tail_dx  = old_tail_dx;
  bool new_tail_br = old_tail_br;

  if (track != nullptr) {
    auto edge = track->get_edge(tail.node_idx, node.node_idx);
    if (!edge.has_value()) {
      _assert(false, "bad edge in path push");
      return false;
    }

    new_tail_dx = edge->dist;
    if (tail.type == NODE_BRANCH) {
      new_tail_br = (track->operator[](tail.node_idx).edge[DIR_CURVED].dest ==
                     &track->operator[](node.node_idx));
    }
  }

  dist_mm      -= old_tail_dx;
  tail.dx_next  = new_tail_dx;
  if (tail.type == NODE_BRANCH) {
    tail.br_curved = new_tail_br;
  }
  dist_mm += tail.dx_next;

  to_insert.dx_next = 0;
  if (!Buffer<PathNode, TRACK_MAX>::push(to_insert)) {
    dist_mm        -= tail.dx_next;
    tail.dx_next    = old_tail_dx;
    tail.br_curved  = old_tail_br;
    dist_mm        += tail.dx_next;
    return false;
  }

  return true;
}

bool Path::push_front(const PathNode &node) {
  PathNode to_insert = node;

  if (empty()) {
    to_insert.dx_next = 0;
    return Buffer<PathNode, TRACK_MAX>::push_front(to_insert);
  }

  auto old_head_opt = peek();
  if (!old_head_opt.has_value()) {
    _assert(false, "non-empty path has no head");
    return false;
  }

  int edge_dist = to_insert.dx_next;
  bool curved   = to_insert.br_curved;

  if (track != nullptr) {
    auto edge = track->get_edge(to_insert.node_idx, old_head_opt->node_idx);
    if (!edge.has_value()) {
      _assert(false, "bad edge in path push_front");
      return false;
    }

    edge_dist = edge->dist;
    if (to_insert.type == NODE_BRANCH) {
      curved = (track->operator[](to_insert.node_idx).edge[DIR_CURVED].dest ==
                &track->operator[](old_head_opt->node_idx));
    }
  }

  to_insert.dx_next = edge_dist;
  if (to_insert.type == NODE_BRANCH) {
    to_insert.br_curved = curved;
  }

  if (!Buffer<PathNode, TRACK_MAX>::push_front(to_insert)) {
    return false;
  }

  dist_mm += edge_dist;
  return true;
}

Path Path::reverse() {
  // start at the back of the path, and reverse the order of nodes
  // we need to use the track ot get hte right edge though
  Path reversed_path{};

  reversed_path.track = track;
  if (empty()) {
    return reversed_path;
  }

  if (track == nullptr) {
    _assert(false, "Path::reverse() called with null track");
    return reversed_path;
  }
  auto &tra = *track;

  reversed_path.track = track;

  for (auto it = end() - 1; it > begin(); --it) {
    auto &node      = *it;
    auto &prev_node = *(it - 1);
    auto edge       = tra.get_edge(prev_node.node_idx, node.node_idx);
    if (!edge.has_value()) {
      _assert(false, "bad edge in path reverse");
      return reversed_path;
    }

    auto new_edge = edge->reverse;
    auto new_node = new_edge->src;

    reversed_path.push({
        .node_idx        = new_node->idx,
        .type            = new_node->type,
        .dx_next         = new_edge->dist,
        .br_curved       = new_node->type == NODE_BRANCH &&
                           new_edge == &new_node->edge[DIR_CURVED],
        .has_reservation = prev_node.has_reservation,
    });
  }

  auto node = tra[(*(begin())).node_idx].reverse;

  reversed_path.push({
      .node_idx        = node->idx,
      .type            = node->type,
      .dx_next         = 0,
      .br_curved       = false,
      .has_reservation = false,
  });

  return reversed_path;
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

namespace {
  template <typename Fn>
  void for_each_outgoing_edge(track_node *track, int idx, Fn &&fn) {
    auto &node = track[idx];
    switch (node.type) {
    case NODE_SENSOR:
    case NODE_MERGE:
    case NODE_ENTER:
      fn(node.edge[DIR_AHEAD]);
      break;
    case NODE_BRANCH:
      fn(node.edge[DIR_STRAIGHT]);
      fn(node.edge[DIR_CURVED]);
      break;
    default:
      break;
    }
  }

  template <typename Fn>
  void walk_reservation_nodes(track_node *track, int start_idx, Fn &&on_node) {
    if (start_idx < 0 || start_idx >= TRACK_MAX) {
      return;
    }

    int stack[TRACK_MAX]    = {0};
    size_t top              = 0;
    bool visited[TRACK_MAX] = {false};

    visited[start_idx] = true;
    stack[top++]       = start_idx;

    while (top > 0) {
      int idx = stack[--top];
      if (idx < 0 || idx >= TRACK_MAX) {
        continue;
      }
      on_node(idx);

      if ((track[idx].reverse != nullptr && !visited[track[idx].reverse->idx] &&
           ((track[idx].reverse->edge[DIR_AHEAD].dist == 0) ||
            track[idx].edge[DIR_AHEAD].dist == 0))) {
        visited[track[idx].reverse->idx] = true;
        stack[top++]                     = track[idx].reverse->idx;
      }

      for_each_outgoing_edge(track, idx, [&](const track_edge &edge) {
        if (edge.dest == nullptr) {
          return;
        }

        if (edge.dest->reverse == nullptr) {
          return;
        }

        int rev_idx = edge.dest->reverse->idx;
        if (!(rev_idx < 0 || rev_idx >= TRACK_MAX || visited[rev_idx])) {
          visited[rev_idx] = true;
          stack[top++]     = rev_idx;
        }
      });
    }
  }
} // namespace

void Track::reserve(int node_idx, uint32_t id) {
  walk_reservation_nodes(track, node_idx,
                         [&](int idx) { track[idx].res_loco_id = id; });
}

void Track::release(int node_idx, uint32_t id) {
  walk_reservation_nodes(track, node_idx, [&](int idx) {
    if (track[idx].res_loco_id == id) {
      track[idx].res_loco_id = UNRESERVED;
    }
  });
}

uint32_t Track::get_reservation(int node_idx) {
  uint32_t owner = UNRESERVED;
  walk_reservation_nodes(track, node_idx, [&](int idx) {
    auto curr_owner = track[idx].res_loco_id;
    if (curr_owner == UNRESERVED) {
      return;
    }

    if (owner == UNRESERVED) {
      owner = curr_owner;
      return;
    }

    if (owner != curr_owner) {
      // Inconsistent ownership across the reserved set means conflict.
      owner = 155;
      return;
    }
  });

  return owner;
}

std::optional<int> Track::get_idx(const Track::NodeName &name) const {
  return node_to_idx.get(name);
}

std::optional<track_edge> Track::get_edge(int from_idx, int to_idx) const {
  const track_node &from = track[from_idx];

  if (from.reverse->idx == to_idx) {
    return std::nullopt;
  }

  switch (from.type) {
  case NODE_SENSOR:
  case NODE_MERGE:
  case NODE_ENTER:
    if (from.edge[DIR_AHEAD].dest->idx == to_idx)
      return from.edge[DIR_AHEAD];
    break;
  case NODE_BRANCH:
    if (from.edge[DIR_STRAIGHT].dest->idx == to_idx)
      return from.edge[DIR_STRAIGHT];
    if (from.edge[DIR_CURVED].dest->idx == to_idx)
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
  result.track = this;

  for (size_t step = 0; step < path_len; ++step) {
    int node_idx           = node_indices[step];
    const track_node &node = track[node_idx];

    int dist_to_next = 0;
    bool curved      = false;

    if (step + 1 < path_len) {
      int next_idx = node_indices[step + 1];
      auto edge    = get_edge(node_idx, next_idx).value_or({});
      dist_to_next = edge.dist;
      if (node.type == NODE_BRANCH) {
        curved = (node.edge[DIR_CURVED].dest == &track[next_idx]);
      }
    }

    result.push({.node_idx  = node_idx,
                 .type      = node.type,
                 .dx_next   = dist_to_next,
                 .br_curved = curved});
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

std::optional<Path> Track::find_path(const Track::NodeName &from,
                                     const Track::NodeName &to,
                                     bool allow_reverse) const {
  auto start_idx = get_idx(from);
  auto goal_idx  = get_idx(to);
  if (!start_idx.has_value() || !goal_idx.has_value())
    return std::nullopt;

  return find_path(start_idx.value(), goal_idx.value(), allow_reverse);
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

StaticString<128> Path::to_string(const Track *track) const {
  StaticString<128> result{};
  result.append("mm:", dist_mm, " ");
  for (const auto &node : *this) {
    result.append((*track)[node.node_idx].name,
                  node.type == NODE_BRANCH ? node.br_curved ? "C" : "S" : "",
                  " >");
  }
  return result;
}

std::optional<PathLocation> Path::locate_at(int offset_um) const {
  for (const auto &node : *this) {
    int seg_um = node.dx_next * 1000;
    if (offset_um < seg_um) {
      return PathLocation{.node_idx  = node.node_idx,
                          .br_curved = node.br_curved};
    }
    offset_um -= seg_um;
  }
  return std::nullopt;
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
                 "  [%d] node_idx=%d name=%s type=%s dist_next=%d "
                 "curved=%d\n\r",
                 static_cast<int>(step), node->node_idx,
                 pathfind.node_name(node->node_idx), node_type_name(node->type),
                 node->dx_next, node->br_curved ? 1 : 0);
  }
}

static int sum_path_dx_mm(const Path &path) {
  int sum = 0;
  for (const auto &node : path) {
    sum += node.dx_next;
  }
  return sum;
}

static void assert_path_dist_consistent(const Path &path) {
  _assert(path.dist_mm == sum_path_dx_mm(path),
          "path dist_mm must match sum(dx_next)");

  auto tail = path.peek_last();
  if (tail.has_value()) {
    _assert(tail->dx_next == 0, "path tail dx_next must be 0");
  }
}

void test_reservations() {
  Track track_c(Track::Layout::B);
  track_c.reserve(track_c["D7"].idx, 1);

  for (int i = 0; i < 139; ++i) {
    if (track_c[i].res_loco_id != UNRESERVED) {
      debug_printf(CONSOLE, "node %s\n\r", track_c[i].name,
                   track_c[i].res_loco_id);
    }
  }
}

void test_pathfind() {
  debug_puts(CONSOLE, "pathfind tests\n\r");

  test_reservations();
  return;

  Track track_a(Track::Layout::A);
  Track track_b(Track::Layout::B);

  for (int i = 0; i < 143; ++i) {
    auto node = track_a[i];
    if (node.type == NODE_BRANCH) {
      if (node.edge[DIR_STRAIGHT].dist !=
          node.edge[DIR_STRAIGHT].reverse->dist) {
        debug_printf(CONSOLE,
                     "track_a branch %s straight dist mismatch: %d vs %d\n\r",
                     node.name, node.edge[DIR_STRAIGHT].dist,
                     node.edge[DIR_STRAIGHT].reverse->dist);
      }
      if (node.edge[DIR_CURVED].dist != node.edge[DIR_CURVED].reverse->dist) {
        debug_printf(CONSOLE,
                     "track_a branch %s curved dist mismatch: %d vs %d\n\r",
                     node.name, node.edge[DIR_CURVED].dist,
                     node.edge[DIR_CURVED].reverse->dist);
      }
    } else if (node.type != NODE_EXIT) {
      if (node.edge[DIR_AHEAD].dist != node.edge[DIR_AHEAD].reverse->dist) {
        debug_printf(CONSOLE,
                     "track_a node %s ahead dist mismatch: %d vs %d\n\r",
                     node.name, node.edge[DIR_AHEAD].dist,
                     node.edge[DIR_AHEAD].reverse->dist);
      }
    }
  }

  for (int i = 0; i < 139; ++i) {
    auto node = track_b[i];
    if (node.type == NODE_BRANCH) {
      if (node.edge[DIR_STRAIGHT].dist !=
          node.edge[DIR_STRAIGHT].reverse->dist) {
        debug_printf(CONSOLE,
                     "track_b branch %s straight dist mismatch: %d vs %d\n\r",
                     node.name, node.edge[DIR_STRAIGHT].dist,
                     node.edge[DIR_STRAIGHT].reverse->dist);
      }
      if (node.edge[DIR_CURVED].dist != node.edge[DIR_CURVED].reverse->dist) {
        debug_printf(CONSOLE,
                     "track_b branch %s curved dist mismatch: %d vs %d\n\r",
                     node.name, node.edge[DIR_CURVED].dist,
                     node.edge[DIR_CURVED].reverse->dist);
      }
    } else if (node.type != NODE_EXIT) {
      if (node.edge[DIR_AHEAD].dist != node.edge[DIR_AHEAD].reverse->dist) {
        debug_printf(CONSOLE,
                     "track_b node %s ahead dist mismatch: %d vs %d\n\r",
                     node.name, node.edge[DIR_AHEAD].dist,
                     node.edge[DIR_AHEAD].reverse->dist);
      }
    }
  }

  // print_path(track_a, "A1->A13", track_a.find_path("A1", "A13", true));
  // print_path(track_a, "A13->A1", track_a.find_path("A13", "A1"));
  // print_path(track_a, "A1->E16", track_a.find_path("A1", "E16"));
  // print_path(track_a, "A1->A1", track_a.find_path("A1", "A1"));
  // print_path(track_a, "A1->ZZZ", track_a.find_path("A1", "ZZZ"));
  // print_path(track_a, "A13->B6", track_a.find_path("A13", "B6"));

  // print_path(track_a, "B6->B6", track_a.find_path("B6", "B6"));
  // print_path(track_a, "B5->B5", track_a.find_path("B5", "B5"));
  // print_path(track_a, "E7->E7", track_a.find_path("E7", "E7"));
  // print_path(track_a, "E8->E8", track_a.find_path("E8", "E8"));

  // print_path(track_b, "C10->B16", track_b.find_path("C10", "B16"));
  // print_path(track_b, "C10->C10", track_b.find_path("C10", "C10"));
  // print_path(track_b, "C13->A11", track_b.find_path(44, 10));
  // print_path(track_b, "B6->B6", track_b.find_path("B6", "B6"));
  // print_path(track_b, "B5->B5", track_b.find_path("B5", "B5"));
  // print_path(track_b, "E7->E7", track_b.find_path("E7", "E7"));
  // print_path(track_b, "E8->E8", track_b.find_path("E8", "E8"));

  auto path_opt = track_b.find_path("C14", "A2");
  auto path     = path_opt.value();

  _assert(path_opt.has_value(), "expected a valid C14->A2 path");
  assert_path_dist_consistent(path);

  print_path(track_b, "C14->A2", path);

  auto node = std::ranges::find(path, path.peek_last());

  debug_puts(CONSOLE, node == path.end() ? "not found\n\r" : "found\n\r");

  auto reversed = path.reverse();
  assert_path_dist_consistent(reversed);
  _assert(reversed.dist_mm == path.dist_mm,
          "reversed path distance must match original");
  print_path(track_b, "A2->C14", reversed);

  // Mutation regression checks: pop/pop_back/pop(n)/clear keep dist_mm synced.
  Path pop_front_test = path;
  auto popped_front   = pop_front_test.pop();
  _assert(popped_front.has_value(), "pop() should return value for non-empty");
  assert_path_dist_consistent(pop_front_test);

  Path pop_back_test = path;
  auto popped_back   = pop_back_test.pop_back();
  _assert(popped_back.has_value(),
          "pop_back() should return value for non-empty");
  assert_path_dist_consistent(pop_back_test);

  Path pop_n_test = path;
  pop_n_test.pop(2);
  assert_path_dist_consistent(pop_n_test);

  Path clear_test = path;
  clear_test.clear();
  _assert(clear_test.empty(), "clear() should make path empty");
  _assert(clear_test.dist_mm == 0, "clear() should reset path distance");

  Path push_front_test{};
  push_front_test.track = &track_b;
  auto e8_opt           = track_b.get_idx("E8");
  auto c14_opt          = track_b.get_idx("C14");
  _assert(e8_opt.has_value() && c14_opt.has_value(),
          "expected E8/C14 nodes for push_front test");
  _assert(push_front_test.push({.node_idx  = c14_opt.value(),
                                .type      = track_b[c14_opt.value()].type,
                                .dx_next   = 123,
                                .br_curved = false}),
          "push should succeed for push_front test");
  _assert(push_front_test.push_front({.node_idx  = e8_opt.value(),
                                      .type      = track_b[e8_opt.value()].type,
                                      .dx_next   = 999,
                                      .br_curved = false}),
          "push_front should succeed for push_front test");
  assert_path_dist_consistent(push_front_test);

  auto p1_opt = track_b.find_path("E8", "C14");
  auto p2_opt = track_b.find_path("C14", "A2");
  print_path(track_b, "E8->C14", p1_opt);
  print_path(track_b, "C14->A2", p2_opt);
  Path concat = p1_opt.value() + p2_opt.value();
  print_path(track_b, "E8->A2", concat);
  assert_path_dist_consistent(concat);
  concat.pop(1);
  assert_path_dist_consistent(concat);
  print_path(track_b, "E8->A2 (after pop)", concat);

  Path concat2 = track_b.find_path("E8", "C14").value() +
                 track_b.find_path("C14", "A2").value();
  concat2.pop_back();
  assert_path_dist_consistent(concat2);
  print_path(track_b, "E8->A2 (after pop back)", concat2);

  print_path(track_b, "E8->A2 (after pop back)", concat2.reverse());

  // EncodedPath ep(track_a.find_path("B6", "B6").value());
  // Path decoded_path = ep.decode(track_a);
  // print_path(track_a, "B6->B6 decoded", decoded_path);

  debug_puts(CONSOLE, "pathfind tests done\n\r");
}
