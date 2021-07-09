#include "xn_search.hpp"

namespace xn {

bool GridNodeCompare::operator()(const GridNode &lhs,
                                 const GridNode &rhs) const {
  if (reverse)
    return (lhs.cost > rhs.cost);
  else
    return (lhs.cost < rhs.cost);
}

std::size_t GridHash::operator()(const GridNode &node) const {
  std::size_t h1 = std::hash<int>()(node.x);
  std::size_t h2 = std::hash<int>()(node.y);
  return h1 ^ h2;
}

int manhattan_distance(GridNode &a, GridNode &b) {
  return abs(b.x - a.x) + abs(b.y - a.y);
}

std::vector<GridNode>
trace_path(std::unordered_map<GridNode, GridNode, GridHash> came_from,
           GridNode current) {
  std::vector<GridNode> out;
  out.push_back(current);
  while (came_from.find(current) != came_from.end()) {
    current = came_from[current];
    out.insert(out.begin(), current);
  }
  return out;
}

std::vector<GridNode> preprocess_graph(std_vec2d<bool> graph) {
  std::vector<GridNode> node_graph;
  for (unsigned int i = 0; i < graph.size(); i++) {
    for (unsigned int j = 0; j < graph[i].size(); j++) {
      GridNode n(i, j);
      if (graph[i][j])
        node_graph.push_back(n);
    }
  }
  //  Any 2 neighbors sharing a column will be at most graph.front().size()
  //  elements away
  auto get_x_neighbor = [&node_graph,
                         &graph](std::vector<GridNode>::iterator &i, int a) {
    if (a > 0 ? (unsigned)i->x < graph.front().size() - 1
              : (unsigned)i->x > 0 && graph[i->x + a][i->y])
      for (auto j = i + a;
           a > 0 ? (j < node_graph.end()) : (j > node_graph.begin()); j += a)
        if (j->x == i->x + a && j->y == i->y) {
          i->neighbors.push_back(&(*(j)));
          break;
        }
  };

  for (auto i = node_graph.begin(); i < node_graph.end(); i++) {
    // row neighbors will always be adjacent in memory
    if ((unsigned)i->y > 0 && graph[i->x][i->y - 1])
      i->neighbors.push_back(&(*(i - 1)));
    if ((unsigned)i->y < graph.size() - 1 && graph[i->x][i->y + 1])
      i->neighbors.push_back(&(*(i + 1)));

    get_x_neighbor(i, -1);
    get_x_neighbor(i, 1);
  }

  return node_graph;
}

std::vector<GridNode> A_star(std::vector<GridNode> &processed_graph,
                             int x_start, int y_start, int goal_x, int goal_y) {
  std::priority_queue<GridNode, std::vector<GridNode>, GridNodeCompare>
      open_set;
  std::vector<GridNode> &open_set_contents = Container(open_set);
  std::unordered_map<GridNode, GridNode, GridHash>
      came_from; // came_from[n] = node immediately preceding n on the
                 // cheapest path
  std::unordered_map<GridNode, int, GridHash>
      g_score; // cost of cheapest path from start to key node, default
               // val=infinity
  GridNode *start = NULL, *goal = NULL;

  for (GridNode &n1 : processed_graph) {
    g_score[n1] = INT_MAX;
    n1.cost = INT_MAX;

    if (x_start == n1.x && y_start == n1.y)
      start = &n1;
    if (goal_x == n1.x && goal_y == n1.y)
      goal = &n1;
  }

  if (start == NULL || goal == NULL)
    return std::vector<GridNode>();

  g_score[*start] = 0;
  start->cost = 0;
  open_set.push(*start);

  while (!open_set.empty()) {
    GridNode current = open_set.top();
    if (current == *goal)
      return trace_path(came_from, current);

    open_set.pop();
    for (GridNode *n_ptr : current.neighbors) {
      GridNode n = *n_ptr;
      int gscore_new = g_score[current] + manhattan_distance(current, n);
      if (gscore_new < g_score[n]) {
        // new best path
        came_from[n] = current;
        g_score[n] = gscore_new;
        n_ptr->cost = g_score[n] + manhattan_distance(*goal, n);

        bool found = false;
        for (auto i = open_set_contents.begin(); i < open_set_contents.end();
             i++) {
          if (*i == *n_ptr) {
            found = true;
            break;
          }
        }
        if (!found)
          open_set.push(*n_ptr);
      }
    }
  }

  // fail case
  return std::vector<GridNode>();
}

std::vector<GridNode> A_star(std_vec2d<bool> graph, GridNode start,
                             GridNode goal) {
  auto pg = preprocess_graph(graph);
  return A_star(pg, start.x, start.y, goal.x, goal.y);
}

glm::ivec2 world_to_grid(const glm::vec3 &in, float gridbox_size,
                         const glm::vec3 &offset) {
  glm::vec2 out(in.x, in.z);
  out.x -= offset.x;
  out.y -= offset.z;
  out /= gridbox_size;
  return (glm::ivec2)glm::floor(out + 0.5f);
}

glm::vec3 grid_to_world(int x, int y, float gridbox_size,
                        const glm::vec3 &offset) {
  glm::vec3 out(x, 0, y);
  out *= gridbox_size;
  out.x += offset.x;
  out.z += offset.z;
  return out;
}

glm::vec2 screen_to_grid(const glm::vec2 &win_size, const glm::vec2 &pixel,
                         float proj_area, float gridbox_size,
                         const glm::vec3 &offset) {
  glm::vec2 out = pixel;
  for (int i = 0; i < 2; i++) {
    out[i] /= (win_size[i] / 2);
    out[i] -= 1;
    out[i] *= abs(proj_area);
  }

  out.x -= offset.x;
  out.y -= offset.z;
  out /= gridbox_size;
  return glm::floor(out + 0.5f);
}
} // namespace xn