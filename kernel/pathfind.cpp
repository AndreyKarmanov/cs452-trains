#include "pathfind.h"
#include "debug.h"
#include "uart.h"
#include <climits>

static constexpr int INF = INT_MAX / 2;

Path &Path::operator+(const Path &other) {
  if (other.empty())
    return *this;

  auto last_opt  = this->peek_last();
  auto first_opt = other.peek();

  if (!(last_opt->node_idx == first_opt->node_idx)) {
    _assert(false, "other must start at last node of this");
    return *this;
  }

  if (this->size() + other.size() - 1 > TRACK_MAX) {
    _assert(false, "Path overflow");
    return *this;
  }

  this->dist += other.dist;
  (*(this->end() - 1)).distance_to_next_node = first_opt->distance_to_next_node;
  for (size_t i = 1; i < other.size(); ++i) {
    auto node = other[i];
    _assert(node.has_value(), "unexpected empty path node");
    this->push(node.value());
  }
  return *this;
}

int Path::lookahead(int distance, PathNode *result, int length,
                    int start_offset, node_type filter_node_type) {
  int count     = 0;
  int travelled = 0;
  for (size_t i = 0; i < size() && count < length; ++i) {
    auto node_opt = (*this)[i];
    if (!node_opt.has_value())
      break;
    if (travelled + start_offset > distance)
      break;

    // skip nodes before start_offset
    if (travelled < start_offset) {
      travelled += node_opt->distance_to_next_node;
      continue;
    }

    travelled += node_opt->distance_to_next_node;

    // filter by node type if provided
    if (filter_node_type != NODE_NONE && node_opt->type != filter_node_type)
      continue;
    result[count++] = *node_opt;
  }
  return count;
}

track_node Pathfind::track[TRACK_MAX];

Pathfind::Pathfind(char track_layout) {
  if (track_layout == 'b')
    init_trackb(track);
  else
    init_tracka(track);

  for (int node_idx = 0; node_idx < TRACK_MAX; ++node_idx) {
    if (track[node_idx].name != nullptr && track[node_idx].name[0] != '\0')
      node_to_idx.set(StaticString<8>(track[node_idx].name), node_idx);
  }
}

std::optional<int> Pathfind::get_idx(const char *name) const {
  StaticString<8> node_name(name);
  return node_to_idx.get(node_name);
}

bool Pathfind::can_visit(int node_idx) const {
  (void)node_idx;
  // use if we want to block particular nodes in the future.
  return true;
}

int Pathfind::edge_dist_between(int from_idx, int to_idx) const {
  const track_node &from    = track[from_idx];
  const track_node &to_node = track[to_idx];

  if (from.reverse == &to_node)
    return 0;

  switch (from.type) {
  case NODE_SENSOR:
  case NODE_MERGE:
  case NODE_ENTER:
    if (from.edge[DIR_AHEAD].dest == &to_node)
      return from.edge[DIR_AHEAD].dist;
    break;

  case NODE_BRANCH:
    if (from.edge[DIR_STRAIGHT].dest == &to_node)
      return from.edge[DIR_STRAIGHT].dist;
    if (from.edge[DIR_CURVED].dest == &to_node)
      return from.edge[DIR_CURVED].dist;
    break;

  default:
    break;
  }

  return 0;
}

void Pathfind::relax(int from_idx, int from_dist, int to_idx, int edge_dist,
                     int best_dist[TRACK_MAX], int predecessor[TRACK_MAX],
                     Heap<std::pair<int, int>, TRACK_MAX> &frontier) const {
  if (!can_visit(to_idx))
    return;

  int new_dist = from_dist + edge_dist;
  if (new_dist < best_dist[to_idx]) {
    best_dist[to_idx]   = new_dist;
    predecessor[to_idx] = from_idx;
    frontier.push({new_dist, to_idx});
  }
}

std::optional<Path>
Pathfind::build_path(int start_idx, int goal_idx,
                     const int best_dist[TRACK_MAX],
                     const int predecessor[TRACK_MAX]) const {
  (void)start_idx;

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
  result.dist = best_dist[goal_idx];

  for (size_t step = 0; step < path_len; ++step) {
    int node_idx           = node_indices[step];
    const track_node &node = track[node_idx];

    int dist_to_prev = 0;
    int dist_to_next = 0;
    bool curved      = false;
    if (step > 0) {
      int prev_idx = node_indices[step - 1];
      dist_to_prev = edge_dist_between(prev_idx, node_idx);
    }
    if (step + 1 < path_len) {
      int next_idx = node_indices[step + 1];
      dist_to_next = edge_dist_between(node_idx, next_idx);
      if (node.type == NODE_BRANCH)
        curved = is_curved(node_idx, next_idx);
    }

    result.push(
        {node_idx, node.type, node.num, dist_to_prev, dist_to_next, curved});
  }

  return result;
}

std::optional<Path> Pathfind::shortest_path(int start_idx, int goal_idx,
                                            bool allow_reverse) const {
  if (start_idx < 0 || start_idx >= TRACK_MAX || goal_idx < 0 ||
      goal_idx >= TRACK_MAX)
    return std::nullopt;

  if (start_idx == goal_idx) {
    const track_node &node = track[start_idx];
    Path result{};
    result.dist = 0;
    result.push({start_idx, node.type, node.num, 0, 0, false});
    return result;
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
      relax(curr_idx, curr_dist, node_index(curr_node.edge[DIR_AHEAD].dest),
            curr_node.edge[DIR_AHEAD].dist, best_dist, predecessor, frontier);
      break;

    case NODE_BRANCH:
      relax(curr_idx, curr_dist, node_index(curr_node.edge[DIR_STRAIGHT].dest),
            curr_node.edge[DIR_STRAIGHT].dist, best_dist, predecessor,
            frontier);
      relax(curr_idx, curr_dist, node_index(curr_node.edge[DIR_CURVED].dest),
            curr_node.edge[DIR_CURVED].dist, best_dist, predecessor, frontier);
      break;

    case NODE_EXIT:
      break;

    default:
      break;
    }

    if (allow_reverse) {
      relax(curr_idx, curr_dist, node_index(curr_node.reverse), REVERSE_COST,
            best_dist, predecessor, frontier);
    }
  }

  return build_path(start_idx, goal_idx, best_dist, predecessor);
}

std::optional<Path> Pathfind::shortest_path(const char *from, const char *to,
                                            bool allow_reverse) const {
  auto start_idx = get_idx(from);
  auto goal_idx  = get_idx(to);
  if (!start_idx.has_value() || !goal_idx.has_value())
    return std::nullopt;

  return shortest_path(start_idx.value(), goal_idx.value(), allow_reverse);
}

bool Pathfind::is_curved(int from_idx, int to_idx) const {
  if (from_idx < 0 || from_idx >= TRACK_MAX || to_idx < 0 ||
      to_idx >= TRACK_MAX)
    return false;

  const track_node &from_node = track[from_idx];
  switch (from_node.type) {
  case NODE_BRANCH:
    if (node_index(from_node.edge[DIR_STRAIGHT].dest) == to_idx)
      return false;
    if (node_index(from_node.edge[DIR_CURVED].dest) == to_idx)
      return true;
    break;

  case NODE_SENSOR:
  case NODE_MERGE:
  case NODE_ENTER:
    return false;
    break;

  default:
    break;
  }

  return false;
}

// returns number of nodes found
int Pathfind::search_within_distance(int node_idx, int distance, int *result,
                                     int length, bool allow_reverse) {
  int best_dist[TRACK_MAX];
  int predecessor[TRACK_MAX];

  for (int i = 0; i < TRACK_MAX; ++i) {
    best_dist[i]   = INF;
    predecessor[i] = -1;
  }

  best_dist[node_idx] = 0;
  Heap<std::pair<int, int>, TRACK_MAX> frontier;
  frontier.push({0, node_idx});

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
      relax(curr_idx, curr_dist, node_index(curr_node.edge[DIR_AHEAD].dest),
            curr_node.edge[DIR_AHEAD].dist, best_dist, predecessor, frontier);
      break;

    case NODE_BRANCH:
      relax(curr_idx, curr_dist, node_index(curr_node.edge[DIR_STRAIGHT].dest),
            curr_node.edge[DIR_STRAIGHT].dist, best_dist, predecessor,
            frontier);
      relax(curr_idx, curr_dist, node_index(curr_node.edge[DIR_CURVED].dest),
            curr_node.edge[DIR_CURVED].dist, best_dist, predecessor, frontier);
      break;

    case NODE_EXIT:
      break;

    default:
      break;
    }

    if (allow_reverse) {
      relax(curr_idx, curr_dist, node_index(curr_node.reverse), REVERSE_COST,
            best_dist, predecessor, frontier);
    }
  }

  return result_count;
}

const char *Pathfind::node_name(int node_idx) const {
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

static void print_path(const Pathfind &pathfind, const char *label,
                       const std::optional<Path> &path_opt) {
  if (!path_opt.has_value()) {
    debug_printf(CONSOLE, "%s: no path\n\r", label);
    return;
  }

  auto path = path_opt.value();

  debug_printf(CONSOLE, "%s: dist=%d len=%d ", label, path.dist,
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
                 node->distance_to_prev_node, node->distance_to_next_node,
                 node->should_br_be_curved ? 1 : 0);
  }
}

void test_pathfind() {
  debug_puts(CONSOLE, "pathfind tests\n\r");

  Pathfind track_a('a');
  print_path(track_a, "A1->A13", track_a.shortest_path("A1", "A13", true));
  print_path(track_a, "A13->A1", track_a.shortest_path("A13", "A1"));
  print_path(track_a, "A1->E16", track_a.shortest_path("A1", "E16"));
  print_path(track_a, "A1->A1", track_a.shortest_path("A1", "A1"));
  print_path(track_a, "A1->ZZZ", track_a.shortest_path("A1", "ZZZ"));
  print_path(track_a, "A13->B6", track_a.shortest_path("A13", "B6"));

  // Pathfind track_b('b');
  // print_path(track_b, "B1->B16", track_b.shortest_path("B1", "B16"));

  debug_puts(CONSOLE, "pathfind tests done\n\r");
}
